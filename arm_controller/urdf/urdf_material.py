from xml.etree.ElementTree import Element


class URDFMaterial:
    name: str
    rgba: list[float]
    texture: str

    def __init__(self, mat: Element, color: Element = None, tex: Element = None):
        self.name = mat.attrib['name']
        if color is not None:
            self.rgba = color.attrib['rgba'].split()
        if tex is not None:
            self.texture = tex.attrib['filename']
