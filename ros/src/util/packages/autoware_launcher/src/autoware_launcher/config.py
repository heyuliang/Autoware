import sys

import os
import xml.etree.ElementTree as ET
import yaml



class AwConfigNode(object):

    def __init__(self, parent, nodepath):
        self.parent   = parent
        self.nodepath = nodepath
        self.children = []
        self.info = {"nodetype":"unknown", "title":"No Title"}

    def isInnerNode(self):
        return self.info["nodetype"] == "node"

    def getNodeName(self):
        return os.path.basename(self.nodepath)

    def dump(self, indent = 0):
        print((" " * indent) + "--------")
        print((" " * indent) + "Path  : " + self.nodepath)
        print((" " * indent) + "Type  : " + self.info["nodetype"])
        print((" " * indent) + "Title : " + self.info["title"])
        for child in self.children:
            child.dump(indent + 2)

    def load_yaml(self, tree):
        fullpath = os.path.join(tree.getTreePath(), self.nodepath + ".yaml")
        try:
            if os.path.isfile(fullpath):
                with open(fullpath) as file:
                    self.info.update(yaml.safe_load(file))
        except:
            print("load yaml error: " + fullpath)

    def load_launch(self, tree):
        fullpath = os.path.join(tree.getTreePath(), self.nodepath + ".xml")
        try:
            root_xml = ET.parse(fullpath).getroot()
            for child_xml in root_xml:
                if child_xml.tag == "include":
                    child_name, child_ext = os.path.splitext(os.path.basename(child_xml.attrib.get("file")))
                    child_node = AwConfigNode(self, os.path.join(self.nodepath, child_name))
                    child_node.load(tree)
                    self.children.append( child_node )
        except:
            print("load launch error: " + fullpath)
    

    def load(self, tree):
        self.load_yaml(tree)
        if self.isInnerNode():
            self.load_launch(tree)



class AwConfigTree(object):

    def __init__(self, treepath):
        self.__rootnode = None
        self.__treepath = treepath

    def getTreePath(self):
        return self.__treepath

    def getRootNode(self):
        return self.__rootnode

    def dump(self):
        print("--------")
        print("Config : " + self.__treepath)
        if self.__rootnode is not None:
            self.__rootnode.dump()

    def load(self):
        self.__rootnode = AwConfigNode(self, "root")
        self.__rootnode.load(self)
        self.dump()



if __name__ == "__main__":
    tree = AwConfigTree(sys.argv[1])
    tree.load()
    tree.dump()
