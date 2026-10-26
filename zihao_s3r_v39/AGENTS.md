<!-- 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9 -->
# V3.9 maintenance

This is the explicitly requested standalone four-mode successor. The original V3.1 continuous control behavior applies to Balance; Rise intentionally retains the user-requested 43-degree gate, delay and final snap. Preserve all four modes.

Read README.md. Keep servo/IMU mappings, policy weights and watchdog unchanged unless requested. UDP target is fixed at 200Hz, default switch off; do not reintroduce a rate selector. The viewer sends subscriptions only, never movement commands. Targets are not measured joints; XY and grounding are display-only.

Keep original checkpoint and header parity, manifest hashes and portable paths. Do not publish serial identities, personal paths, credentials, .venv, caches, ELF/map or private network information. Validate compilation and viewer --check. Real-device uploads require explicit user authorization. Do not claim reliable all-direction Walk or measured absolute latency.
