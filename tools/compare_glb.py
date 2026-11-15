#!/usr/bin/env python3
"""Compare sldprt-glb GLB output with a reference GLB.

usage: compare_glb.py <ours.glb> <ref.glb>

Prints `identical` for byte-identical files. Otherwise prints the first JSON
differences (path, ours, reference) and, for every accessor, the largest
absolute difference of its decoded values. Exit status: 0 identical,
1 different, 2 unreadable.
"""
import json
import struct
import sys

COMPONENTS = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
FORMATS = {5126: 'f', 5125: 'I', 5123: 'H', 5121: 'B'}


def parts(blob):
    if len(blob) < 28 or blob[:4] != b'glTF':
        raise ValueError('not a GLB')
    size, = struct.unpack_from('<I', blob, 12)
    doc = json.loads(blob[20:20 + size])
    end = 20 + size
    binary_size, = struct.unpack_from('<I', blob, end)
    return doc, blob[end + 8:end + 8 + binary_size]


def json_diff(a, b, path='$', out=None, limit=20):
    out = [] if out is None else out
    if len(out) >= limit:
        return out
    if type(a) is not type(b) and not (isinstance(a, (int, float)) and isinstance(b, (int, float))):
        out.append((path, a, b))
    elif isinstance(a, dict):
        if list(a) != list(b):
            out.append((path + ' keys', list(a), list(b)))
        for k in a:
            if k in b:
                json_diff(a[k], b[k], f'{path}.{k}', out, limit)
    elif isinstance(a, list):
        if len(a) != len(b):
            out.append((path + ' len', len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)):
            json_diff(x, y, f'{path}[{i}]', out, limit)
    elif a != b or type(a) is not type(b):
        out.append((path, a, b))
    return out


def accessor_values(doc, binary, index):
    a = doc['accessors'][index]
    view = doc['bufferViews'][a['bufferView']]
    fmt = FORMATS[a['componentType']]
    n = COMPONENTS[a['type']] * a['count']
    start = view.get('byteOffset', 0) + a.get('byteOffset', 0)
    return struct.unpack_from(f'<{n}{fmt}', binary, start)


def main():
    ours_blob, ref_blob = (open(p, 'rb').read() for p in sys.argv[1:3])
    if ours_blob == ref_blob:
        print('identical')
        return 0
    try:
        ours, ours_bin = parts(ours_blob)
        ref, ref_bin = parts(ref_blob)
    except (ValueError, struct.error, json.JSONDecodeError) as e:
        print('unreadable', e)
        return 2
    diffs = json_diff(ours, ref)
    print(f'json: {"equal" if not diffs else f"{len(diffs)}+ differences"}; '
          f'bin: {"equal" if ours_bin == ref_bin else f"{len(ours_bin)} vs {len(ref_bin)} bytes"}')
    for path, a, b in diffs:
        print(f'  {path}: ours={json.dumps(a)[:120]} ref={json.dumps(b)[:120]}')
    if len(ours.get('accessors', [])) == len(ref.get('accessors', [])):
        for i in range(len(ref['accessors'])):
            try:
                x, y = accessor_values(ours, ours_bin, i), accessor_values(ref, ref_bin, i)
            except (KeyError, struct.error):
                print(f'  accessor {i}: unreadable')
                continue
            if len(x) != len(y):
                print(f'  accessor {i}: count {len(x)} vs {len(y)}')
            elif x != y:
                worst = max(abs(p - q) for p, q in zip(x, y))
                print(f'  accessor {i}: max |diff| {worst:.3g} over {sum(p != q for p, q in zip(x, y))} values')
    return 1


if __name__ == '__main__':
    sys.exit(main())
