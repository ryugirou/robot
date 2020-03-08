#!/usr/bin/env python3
### coding: UTF-8
import os
import glob

def replace(path):
  out = ''
  with open(path) as fh:
    for line in fh:
        out += '{' + line.rstrip()[0:-1] + '},\n'
  return out[0:-2]

def write(path,data):
  with open(path,'w') as fh:
    fh.write(data)

def replace_and_write(path):
  if os.path.exists(path):
    write(path,replace(path))
  else:
    print('error')

os.chdir('..')
for path in glob.glob('include/path/**/*.csv',recursive=True):
  replace_and_write(path)
