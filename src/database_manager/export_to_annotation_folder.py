#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import datetime
import os

import cv2
import numpy as np

from database_manager.database import DatabaseClient


def dc_test():
    collection = "InterRow-CloseMergeAndWide"
    db_client = DatabaseClient()

    unannotated_data = [
        {
            '$match': {
                "camera_data": {"$exists": True},
                "annotation": {"$exists": False},
            }
        }
    ]

    data_cursor = db_client.export_dict(collection, unannotated_data)

    if not os.path.isdir("unannotated_data"):
        os.makedirs("unannotated_data")

    for document in data_cursor:
        # print('_id' in document, 'annotation' in document)
        if document["camera_data"]["camera_name"] != 'rdc_back_wide':
            continue

        rgb_sensor = document["camera_data"]["rgb"]
        rgb_image = np.frombuffer(rgb_sensor["data"], dtype=rgb_sensor["dtype"]).reshape(rgb_sensor["shape"])
        rgb_filename = os.path.join("unannotated_data", "{}.png".format(document["_id"]))

        if os.path.exists(rgb_filename):
            print("File Exists:", rgb_filename)
        else:
            print("Writing File:", rgb_filename)
            cv2.imwrite(rgb_filename, rgb_image[..., ::-1])

    return
    # dates = sorted(list(set([x["doc_datetime"] for x in data_cursor])))
    # for date in dates:
    #     pipeline_at_date = [{'$match': {'doc_datetime': date}}]
    #     data_cursor = db_client.export_dict(collection, pipeline_at_date)
    #     images = []
    #     for data in data_cursor:
    #         rgb_sensor = data["camera_data"]["rgb"]
    #         rgb_image = np.frombuffer(rgb_sensor["data"], dtype=rgb_sensor["dtype"]).reshape(rgb_sensor["shape"])
    #         images.append(rgb_image[..., ::-1])
    #     cv2.destroyAllWindows()
    #     cv2.imshow("Test", np.vstack(images))
    #     cv2.waitKey(0)


if __name__ == '__main__':
    dc_test()
