# !/usr/bin/env python
from __future__ import absolute_import, division, print_function

import fnmatch
import os
import pickle

from database_manager.database import DatabaseClient


def prompt_yn(action_description=''):
    return raw_input("Would you like to {}? [Y/n]".format(action_description)).lower() in ['yes', 'y', 'ye', '']


def main():
    db_client = DatabaseClient()
    context = "DefaultScenario"
    root_data_folder = "data"
    folder_to_convert = os.path.join(os.path.abspath(root_data_folder), context)
    files = []

    for root, _, filenames in os.walk(folder_to_convert):
        for filename in fnmatch.filter(filenames, '*.pkl'):
            files.append(os.path.join(root, filename))

    for file in files:
        with open(file, "r") as fh:
            doc = pickle.load(fh)
            if prompt_yn("insert object from time {} to the database".format(doc.doc_datetime)):
                db_client.import_dict(collection=context, dictionary=doc.to_dict())
    print(files)


if __name__ == '__main__':
    main()
