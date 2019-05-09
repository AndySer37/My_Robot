import glob
import random
import os
import os.path as osp
import sys
import numpy as np
import cv2
import torch
import torch.nn.functional as F
from augmentations import horisontal_flip
from torch.utils.data import Dataset
import torchvision.transforms as transforms
if sys.version_info[0] == 2:
    import xml.etree.cElementTree as ET
else:
    import xml.etree.ElementTree as ET

subt_CLASSES =  [  # always index 0
    'bb_extinguisher', 'bb_drill', 'bb_backpack']

def pad_to_square(img, pad_value):
    h, w, _ = img.shape
    dim_diff = np.abs(h - w)
    # (upper / left) padding and (lower / right) padding
    pad1 = dim_diff // 2
    pad2 = dim_diff - pad1
    # Determine padding
    pad = ((pad1, pad2), (0, 0), (0, 0)) if h <= w else ((0, 0), (pad1, pad2), (0, 0))
    # Add padding
    img = np.pad(img, pad, "constant", constant_values=pad_value)

    return img, pad


def random_resize(images, min_size=288, max_size=448):
    new_size = random.sample(list(range(min_size, max_size + 1, 32)), 1)[0]
    images = F.interpolate(images, size=new_size, mode="nearest")
    return images


class ImageFolder(Dataset):
    def __init__(self, folder_path, img_size=416):
        self.files = sorted(glob.glob("%s/*.*" % folder_path))
        self.img_size = img_size

    def __getitem__(self, index):
        img_path = self.files[index % len(self.files)]
        # Extract image
        img = cv2.imread(img_path)
        img, _ = pad_to_square(img, 127)
        # Resize
        img = cv2.resize(img, (self.img_size, self.img_size), interpolation=cv2.INTER_NEAREST)
        # Channels-first
        img = np.transpose(img, (2, 0, 1))
        # As pytorch tensor
        img = torch.from_numpy(img).float() / 255.0

        return img_path, img

    def __len__(self):
        return len(self.files)


class ListDataset(Dataset):
    def __init__(self, list_path, image_sets=[('train')], img_size=416, training=True, augment=True):
        rootpath = "/home/andyser/data/subt_real"
        self.class_to_ind = dict(zip(subt_CLASSES, range(len(subt_CLASSES))))
        self.img_size = img_size
        self.max_objects = 100
        self.is_training = training
        self.augment = augment and training
        self._annopath = osp.join('%s', 'Annotations', '%s.xml')
        self._imgpath = osp.join('%s', 'image', '%s.jpg')
        self.ids = list()
        for name in image_sets:
            for line in open(osp.join(rootpath, 'ImageSets/Main', name + '.txt')):
                self.ids.append((rootpath, line.strip().split(' ')[0]))

    def __getitem__(self, index):

        # ---------
        #  Image
        # ---------
        img_id = self.ids[index]
        target = ET.parse(self._annopath % img_id).getroot()
        
        img = cv2.imread(self._imgpath % img_id)

        # Handles images with less than three channels
        try:
            if len(img.shape) != 3:
                img = np.expand_dims(img, -1)
                img = np.repeat(img, 3, -1)
        except:
            print("=========================", self._imgpath % img_id)

        h, w, _ = img.shape
        img, pad = pad_to_square(img, 127.5)
        padded_h, padded_w, _ = img.shape
        # Resize to target shape
        img = cv2.resize(img, (self.img_size, self.img_size))
        # Channels-first and normalize
        img = torch.from_numpy(img).float().permute((2, 0, 1)) / 255.0

        # ---------
        #  Label
        # ---------

        labels = []
        for obj in target.iter('object'):
            name = obj.find('name').text.lower().strip()
            if name not in self.class_to_ind:
                continue
            polygons = obj.find('polygon')
            x = []
            y = []  
            for polygon in polygons.iter('pt'):
                # scale height or width
                x.append(int(polygon.find('x').text))
                y.append(int(polygon.find('y').text))
            x1 = min(x)
            x2 = max(x)
            y1 = min(y)
            y2 = max(y)  

            x1 += pad[1][0]
            y1 += pad[0][0]
            x2 += pad[1][1]
            y2 += pad[0][1]

            if self.is_training:
                # Returns (x, y, w, h)
                labels.append([self.class_to_ind[name] ,((x1 + x2) / 2) / padded_w, ((y1 + y2) / 2) / padded_h, (x2 - x1)/padded_w, (y2 - y1)/padded_h])
                # labels[:, 1] = ((x1 + x2) / 2) / padded_w
                # labels[:, 2] = ((y1 + y2) / 2) / padded_h
                # labels[:, 3] *= w / padded_w
                # labels[:, 4] *= h / padded_h    # (y2 - y1)/padded_h
            else:
                labels.append([self.class_to_ind[name] ,x1 * (self.img_size / padded_w), y1 * (self.img_size / padded_h), x2 * (self.img_size / padded_w), y2 * (self.img_size / padded_h)])
                # Returns (x1, y1, x2, y2)
                # labels[:, 1] = x1 * (self.img_size / padded_w)
                # labels[:, 2] = y1 * (self.img_size / padded_h)
                # labels[:, 3] = x2 * (self.img_size / padded_w)
                # labels[:, 4] = y2 * (self.img_size / padded_h)

        # Apply augmentations
        if self.augment:
            if np.random.random() < 0.5:
                img, labels = horisontal_flip(img, labels)

        # Add dummy label if there are none
        num_labels = 1 if labels is None else len(labels)
        boxes = torch.zeros((num_labels, 6))
        if labels is not None:
            boxes[:, 1:] = torch.FloatTensor(labels)

        return self._imgpath % img_id, img, boxes

    @staticmethod
    def collate_fn(batch):
        paths, imgs, labels = list(zip(*batch))
        for i, boxes in enumerate(labels):
            boxes[:, 0] = i
        imgs = torch.stack(imgs, 0)
        labels = torch.cat(labels, 0)
        return paths, imgs, labels

    def __len__(self):
        return len(self.ids)
