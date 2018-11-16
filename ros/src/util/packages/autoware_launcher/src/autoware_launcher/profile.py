import sys
import xml.etree.ElementTree as ET

class AwProfileNode(object):

    def __init__(self):
        self.title = 'Untitled Profile'
        self.children = []

    def dump(self, indent = 0):
        print((' ' * indent) + self.title)
        for child in self.children:
            child.dump(indent + 2)

def load(path):
    return _load_node(path, 'root.launch')

def _load_node(path, file):
    profile = AwProfileNode()
    profile.title = file
    root = ET.parse(path + file).getroot()
    for child in root:
        profile.children.append( _load_node(path, child.attrib.get('file')) )
    return profile

if __name__ == '__main__':
    profile = load(sys.argv[1])
    profile.dump()
    