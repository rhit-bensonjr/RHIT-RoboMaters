# -*- coding: utf-8 -*-
# Copyright (c) 2013-2016, Niklas Hauser
# Copyright (c)      2016, Fabian Greif
# All rights reserved.

import os
import logging

from lxml import etree

from ..device_tree import DeviceTree

LOGGER = logging.getLogger('dfg.output.xml')

class DeviceFileWriter:
    """ DeviceFileWriter
    Formats a generic tree as a modm device file.
    """

    @staticmethod
    def toEtree(tree):
        assert tree.name == 'device'

        # Add the RCA root element
        root = etree.Element('modm')
        root.set('version', '0.4.0')
        root.append(etree.Comment(" WARNING: This file is generated by the modm device file generator. Do not edit! "))
        # Format the entire tree
        device = etree.SubElement(root, tree.name)
        # Add the naming schema
        schema = etree.Element('naming-schema')
        schema.text = tree.ids.naming_schema
        device.append(schema)
        # compute all invalid identifier string combinations
        invalid = sorted([did.string for did in tree.ids.product() if did not in tree.ids])
        valid = sorted([did.string for did in tree.ids])
        # choose the shorter list
        invalid_list = (len(invalid) < len(valid))
        for did in (invalid if invalid_list else valid):
            dev = etree.Element('invalid-device' if invalid_list else 'valid-device')
            dev.text = did
            device.append(dev)

        DeviceFileWriter._to_etree(tree, device)
        return root

    @staticmethod
    def _to_etree(tree, root):
        root_ids = tree.ids
        for k in root_ids.keys():
            root.set(k, "|".join(root_ids.getAttribute(k)))
        for child in tree.children:
            DeviceFileWriter._to_etree_iter(root_ids, child, root)

    @staticmethod
    def _to_etree_iter(root_ids, tree, parent):
        diffs = tree.ids.minimal_subtract_set(root_ids, tree.parent.ids)

        for diff in diffs:
            me = etree.SubElement(parent, tree.name)
            # sort diff keys in root_id order
            rootkeys = root_ids.keys()
            diffkeys = [(rootkeys.index(d), d) for d in diff.keys()]
            diffkeys = [k for (_, k) in sorted(diffkeys, key=lambda d: d[0])]

            for k in diffkeys:
                v = diff.getAttribute(k)
                if len(v) > 0:
                    me.set('device-' + k, "|".join(v))

            for k,v in tree.attributes.items():
                if k != 'value':
                    me.set(k, v)
            if tree['value'] is not None:
                me.set('value', tree['value'])

            for child in tree.children:
                DeviceFileWriter._to_etree_iter(root_ids, child, me)

    @staticmethod
    def format(tree):
        return etree.tostring(DeviceFileWriter.toEtree(tree),
                              encoding="UTF-8",
                              pretty_print=True,
                              xml_declaration=True)

    @staticmethod
    def write(tree, folder, name):
        path = os.path.join(str(folder), name(tree.ids) + '.xml')
        content = DeviceFileWriter.format(tree).decode('utf-8')

        if os.path.exists(path):
            LOGGER.warning("Overwriting file '%s'", os.path.basename(path))
        else:
            LOGGER.info("New XML file: '%s'", os.path.basename(path))
        with open(path, 'w') as device_file:
            device_file.write(content)
        return path
