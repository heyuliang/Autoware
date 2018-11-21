import sys

import os
import xml.etree.ElementTree as ET
import yaml



class AwConfigNode(object):

    def __init__(self):
        self.nodepath = ""
        self.children = []
        self.info = False
        self.title = "[No information]"

    def dump(self, indent = 0):
        print((" " * indent) + "--------")
        print((" " * indent) + "Title : " + self.title)
        print((" " * indent) + "Path  : " + self.nodepath)
        for child in self.children:
            child.dump(indent + 2)

class AwConfigTree(object):

    def __init__(self, treepath):
        self.rootnode = None
        self.treepath = treepath

    def dump(self):
        if self.rootnode is not None:
            self.rootnode.dump()

    def load(self):
        self.rootnode = self.load_node("root")

    def load_node(self, nodepath):

        node = AwConfigNode()
        node.nodepath = nodepath

        # Read information file
        try:
            fullpath = self.treepath + nodepath + ".yaml"
            if os.path.isfile(fullpath):
                with open(fullpath) as file:
                    info = yaml.safe_load(file)
                    node.info = True
                    node.title = info["title"]
        except:
            print("yaml open error")
            pass

        # Read launch file
        if node.info:
            fullpath = self.treepath + nodepath + ".launch"
            root_xml = ET.parse(fullpath).getroot()
            for child_xml in root_xml:
                if child_xml.tag == "include":
                    child_name, child_ext = os.path.splitext(os.path.basename(child_xml.attrib.get("file")))
                    node.children.append( self.load_node(nodepath + os.path.sep + child_name) )

        return node


if __name__ == "__main__":
    tree = AwConfigTree(sys.argv[1])
    tree.load()
    tree.dump()
