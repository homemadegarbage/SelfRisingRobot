# 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
"""Visual-only MuJoCo dark grid, generated from the source model at load time."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco

def load_dark_model(path):
    path=Path(path).resolve();root=ET.parse(path).getroot()
    comp=root.find('compiler')
    if comp is not None:
        for key in ['meshdir','texturedir']:
            comp.set(key,str((path.parent/comp.get(key,'')).resolve()))
    asset=root.find('asset')
    if asset is None:asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'texture',name='live_grid',type='2d',builtin='flat',rgb1='.035 .045 .055',rgb2='.035 .045 .055',mark='edge',markrgb='.65 .72 .78',width='128',height='128')
    ET.SubElement(asset,'material',name='live_floor',texture='live_grid',texrepeat='20 20',texuniform='true',reflectance='0',specular='.1',shininess='.1')
    floor=root.find(".//geom[@name='ground_visual']")
    if floor is not None:floor.set('material','live_floor');floor.set('rgba','1 1 1 1')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
