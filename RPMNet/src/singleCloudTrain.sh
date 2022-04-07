#!/bin/sh
python train.py --dataset_type modelnet_hdf --dataset_path ../datasets/modelnet40_ply_hdf5_2048 --train_categoryfile ./data_loader/modelnet40_half1.txt --val_categoryfile ./data_loader/modelnet40_half1.txt
