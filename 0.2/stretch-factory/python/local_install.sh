#! /bin/bash

mkdir -p $HOME/.local/lib/python2.7/site-packages/stretch_factory
mkdir -p $HOME/.local/lib/python3.6/site-packages/stretch_factory

cp -rf stretch_factory/* $HOME/.local/lib/python2.7/site-packages/stretch_factory
echo "Copied python to $HOME/.local/lib/python2.7/site-packages/stretch_factory/"
cp -rf stretch_factory/* $HOME/.local/lib/python3.6/site-packages/stretch_factory
echo "Copied python to $HOME/.local/lib/python3.6/site-packages/stretch_factory/"
cp tools/* $HOME/.local/bin

