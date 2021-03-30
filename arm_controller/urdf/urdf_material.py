class URDFMaterial:
    name: str
    rgba: list[float, float, float, float]

    def __init__(self, mat, color):
        self.name = mat.attrib['name']
        self.rgba = color.attrib['rgba'].split()
