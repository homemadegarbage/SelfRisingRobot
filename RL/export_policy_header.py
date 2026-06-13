import argparse
import re

from stable_baselines3 import PPO


MODEL_PATH = "robo1_getup_ppo.zip"
OUTPUT_HEADER = "policy_network.h"


def is_policy_key(key: str) -> bool:
    return ("policy_net" in key) or ("action_net" in key)


def layer_order_index(key: str) -> int:
    if "policy_net" in key:
        match = re.search(r"policy_net\.(\d+)", key)
        if match:
            return int(match.group(1))
    if "action_net" in key:
        return 10**6

    numbers = re.findall(r"\d+", key)
    return int(numbers[-1]) if numbers else 10**9


def sanitize(key: str) -> str:
    return key.replace(".", "_").replace("[", "_").replace("]", "_")


def extract_policy_layers(state_dict):
    layers_by_index = {}

    for key, value in state_dict.items():
        if not is_policy_key(key):
            continue

        if key.endswith(".weight") or key.endswith(".bias"):
            index = layer_order_index(key)
            if index not in layers_by_index:
                layers_by_index[index] = {"W": None, "b": None}

            if key.endswith(".weight"):
                layers_by_index[index]["W"] = value.detach().cpu().numpy()
                layers_by_index[index]["w_raw_key"] = key
            else:
                layers_by_index[index]["b"] = value.detach().cpu().numpy()
                layers_by_index[index]["b_raw_key"] = key

    layers = []
    for index in sorted(layers_by_index.keys()):
        entry = layers_by_index[index]
        weight = entry["W"]
        bias = entry["b"]
        if weight is None or bias is None:
            continue

        layers.append(
            {
                "w_name": sanitize(entry["w_raw_key"]) + "_weight",
                "b_name": sanitize(entry["b_raw_key"]) + "_bias",
                "W": weight,
                "b": bias,
            }
        )

    return layers


def write_array(file, name, arr):
    if arr.ndim == 2:
        rows, cols = arr.shape
        file.write(f"static const float {name}[{rows}][{cols}] = {{\n")
        for row in arr:
            file.write("    {" + ", ".join(f"{x:.8f}" for x in row) + "},\n")
        file.write("};\n\n")
    else:
        file.write(f"static const float {name}[{arr.shape[0]}] = {{\n")
        file.write(", ".join(f"{x:.8f}" for x in arr))
        file.write("};\n\n")


def generate_forward(layers):
    code = []
    code.append("// Policy-only MLP forward pass generated from Stable-Baselines3 PPO.\n")
    code.append("#include <math.h>\n")
    code.append("static inline float tanhf_fast(float x) { return tanhf(x); }\n")
    code.append("static inline void forward_policy(const float input[], float output[]) {")
    code.append("    const float *curr = input;")

    for index, layer in enumerate(layers):
        w_name = layer["w_name"]
        b_name = layer["b_name"]
        weight = layer["W"]
        out_dim, in_dim = weight.shape

        code.append(f"    static float layer{index}[{out_dim}];")
        code.append(f"    for (int i = 0; i < {out_dim}; i++) {{")
        code.append(f"        float s = {b_name}[i];")
        code.append(f"        for (int j = 0; j < {in_dim}; j++) {{")
        code.append(f"            s += {w_name}[i][j] * curr[j];")
        code.append("        }")

        if index < len(layers) - 1:
            code.append(f"        layer{index}[i] = tanhf_fast(s);")
        else:
            code.append(f"        layer{index}[i] = s;")

        code.append("    }")
        code.append(f"    curr = layer{index};")

    final_dim = layers[-1]["W"].shape[0]
    code.append(f"    for (int i = 0; i < {final_dim}; i++) output[i] = curr[i];")
    code.append("}\n")
    return "\n".join(code)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export a Stable-Baselines3 PPO policy to a C header."
    )
    parser.add_argument(
        "model_path",
        nargs="?",
        default=MODEL_PATH,
        help=f"source Stable-Baselines3 model zip file (default: {MODEL_PATH})",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=OUTPUT_HEADER,
        help=f"output header file (default: {OUTPUT_HEADER})",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("Loading model:", args.model_path)
    model = PPO.load(args.model_path)
    layers = extract_policy_layers(model.policy.state_dict())
    if not layers:
        raise RuntimeError("No policy_net/action_net layers were found.")

    input_dim = layers[0]["W"].shape[1]
    output_dim = layers[-1]["W"].shape[0]

    with open(args.output, "w") as file:
        file.write("// Auto-generated policy-only MLP\n\n")
        file.write("#pragma once\n\n")
        file.write(f"#define OBS_DIM {input_dim}\n")
        file.write(f"#define ACTION_DIM {output_dim}\n\n")

        for layer in layers:
            write_array(file, layer["w_name"], layer["W"])
            write_array(file, layer["b_name"], layer["b"])

        file.write(generate_forward(layers))

    print("Generated:", args.output)


if __name__ == "__main__":
    main()
