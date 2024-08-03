#!/bin/bash
echo "export PYTHONPATH="$(dirname `pwd`)":$PYTHONPATH" >> ~/.bashrc
