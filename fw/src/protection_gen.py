#!/usr/bin/env python3

for i in range(256):
    print(f'0xFF, // {i} ' + format(i, '#010b'))
