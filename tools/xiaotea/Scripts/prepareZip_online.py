#!/usr/bin/python
from sys import argv, exit
from os.path import getsize
from xiaotea import XiaoTea
import zipfile
import hashlib
import time
import shutil
import os

cry = XiaoTea()

source='build/firmware.bin'
destination='tools/zip_output/FIRM.bin'
shutil.copyfile(source, destination)


hfi = open('tools/zip_output/FIRM.bin', 'rb')
hfo = open('tools/zip_output/FIRM.bin.enc', 'wb')

hfo.write(cry.encrypt(hfi.read()))

hfo.close()
hfi.close()

md5_hash = hashlib.md5()
with open('tools/zip_output/FIRM.bin',"rb") as f:
    # Read and update hash in chunks of 4K
    for byte_block in iter(lambda: f.read(4096),b""):
        md5_hash.update(byte_block)
    print(md5_hash.hexdigest())

md5_hash_enc = hashlib.md5()
with open('tools/zip_output/FIRM.bin.enc',"rb") as f:
    # Read and update hash in chunks of 4K
    for byte_block in iter(lambda: f.read(4096),b""):
        md5_hash_enc.update(byte_block)
    print(md5_hash_enc.hexdigest())

version = 'M365_v3_0.1'
#filename = version + '-' + str(int(time.time())) + '.zip'
#zip_file = zipfile.ZipFile(filename, 'a', zipfile.ZIP_DEFLATED, False)

#zip_file.write('FIRM.bin')

#zip_file.write('FIRM.bin.enc')


info_txt = 'dev: M365;\nnam: {};\nenc: B;\ntyp: DRV;\nmd5: {};\nmd5e: {};\n'.format(
    version, md5_hash.hexdigest(), md5_hash_enc.hexdigest())
infofile = open('tools/zip_output/info.txt', 'wb')
infofile.write(info_txt.encode())
infofile.close()
#os.remove('FIRM.bin')
#os.remove('FIRM.bin.enc')
