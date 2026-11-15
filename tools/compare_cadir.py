#!/usr/bin/env python3
"""Compare sldprt-glb `dump` JSON with a reference cadmpeg CADIR.

usage: compare_cadir.py OURS.json REF.cadir.json [--only tessellations,appearance,presentation,brep,assembly]

Checks what the GLB writer consumes. B-rep entity ids may differ
between implementations, so B-rep content is compared structurally (counts,
geometry kinds and the face/body colour seen by each tessellation).
"""
import argparse
import collections
import json
import math
import sys


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def close(a, b, tol):
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


def vec_close(a, b, tol):
    return all(close(a[k], b[k], tol) for k in ('x', 'y', 'z'))


class Report:
    def __init__(self):
        self.failures = collections.defaultdict(list)

    def fail(self, area, message):
        self.failures[area].append(message)

    def print(self, limit=12):
        if not self.failures:
            print('OK')
            return 0
        for area, messages in self.failures.items():
            print(f'[{area}] {len(messages)} difference(s)')
            for m in messages[:limit]:
                print('   ', m)
        return 1


def tess_colors(model):
    """Face / body colour each tessellation would see (the reference writer table_color order)."""
    faces = {f['id']: f for f in model.get('faces', [])}
    bodies = {b['id']: b for b in model.get('bodies', [])}
    out = {}
    for t in model.get('tessellations', []):
        owned = [faces.get(k, {}).get('color') for k in t.get('faces', [])]
        color = next((c for c in owned if c), None) or bodies.get(t.get('body'), {}).get('color')
        out[t['id']] = (color, bool(t.get('body')), len(t.get('faces', [])))
    return out


def compare_tessellations(ours, ref, rep, tol):
    a = {t['id']: t for t in ours.get('tessellations', [])}
    b = {t['id']: t for t in ref.get('tessellations', [])}
    if list(a) != list(b):
        missing = [k for k in b if k not in a]
        extra = [k for k in a if k not in b]
        rep.fail('tessellations', f'ids differ: {len(a)} vs {len(b)}; missing {missing[:5]} extra {extra[:5]}')
        if [k for k in a if k in b] != [k for k in b if k in a]:
            rep.fail('tessellations', 'order of common ids differs')
    for key in b:
        if key not in a:
            continue
        ta, tb = a[key], b[key]
        ma, mb = ta.get('mesh', {}), tb.get('mesh', {})
        if ma.get('kind') != mb.get('kind'):
            rep.fail('tessellations', f'{key}: mesh kind {ma.get("kind")} vs {mb.get("kind")}')
        sa, sb = ma.get('strips'), mb.get('strips')
        if sa is not None and sb is not None:
            if [len(s) for s in sa] != [len(s) for s in sb]:
                rep.fail('tessellations', f'{key}: strip lengths differ')
            else:
                for strip_a, strip_b in zip(sa, sb):
                    bad = next((i for i, (va, vb) in enumerate(zip(strip_a, strip_b))
                                if not vec_close(va['position'], vb['position'], tol)
                                or ('normal' in vb and not vec_close(va.get('normal', {'x': 0, 'y': 0, 'z': 0}), vb['normal'], tol))), None)
                    if bad is not None:
                        rep.fail('tessellations', f'{key}: vertex {bad} differs: {strip_a[bad]} vs {strip_b[bad]}')
                        break
        so_a, so_b = ta.get('source_object') or {}, tb.get('source_object') or {}
        for field in ('object_id', 'visible', 'color'):
            if so_a.get(field) != so_b.get(field):
                rep.fail('assembly', f'{key}: source_object.{field} {so_a.get(field)!r} vs {so_b.get(field)!r}')


def compare_ownership(ours, ref, rep):
    ca, cb = tess_colors(ours), tess_colors(ref)
    for key, (color_b, body_b, nfaces_b) in cb.items():
        if key not in ca:
            continue
        color_a, body_a, nfaces_a = ca[key]
        if (body_a, nfaces_a) != (body_b, nfaces_b):
            rep.fail('ownership', f'{key}: owned body={body_a} faces={nfaces_a} vs body={body_b} faces={nfaces_b}')
        elif color_a != color_b:
            rep.fail('ownership', f'{key}: owner colour {color_a} vs {color_b}')


def compare_appearance(ours, ref, rep):
    a = {x['id']: x for x in ours.get('appearances', [])}
    b = {x['id']: x for x in ref.get('appearances', [])}
    for key, x in b.items():
        if key not in a:
            rep.fail('appearance', f'missing appearance {key} ({x.get("schema")})')
        elif a[key].get('base_color') != x.get('base_color'):
            rep.fail('appearance', f'{key}: base_color {a[key].get("base_color")} vs {x.get("base_color")}')
    for key in a:
        if key not in b:
            rep.fail('appearance', f'extra appearance {key}')
    ba = {x['id']: x for x in ours.get('appearance_bindings', [])}
    bb = {x['id']: x for x in ref.get('appearance_bindings', [])}
    for key, x in bb.items():
        y = ba.get(key)
        if y is None:
            rep.fail('appearance', f'missing binding {key} -> {x.get("target")}')
            continue
        for field in ('target', 'appearance'):
            if y.get(field) != x.get(field):
                rep.fail('appearance', f'{key}: {field} {y.get(field)} vs {x.get(field)}')
    for key in ba:
        if key not in bb:
            rep.fail('appearance', f'extra binding {key}')


def attr_values(model, name):
    return [a['values'] for a in model.get('attributes', []) if a.get('name') == name]


def compare_presentation(ours, ref, rep, tol):
    for name in ('saved_display_mode', 'saved_reference_plane'):
        va, vb = attr_values(ours, name), attr_values(ref, name)
        if len(va) != len(vb):
            rep.fail('presentation', f'{name}: {len(va)} vs {len(vb)} attribute(s)')
            continue
        for i, (x, y) in enumerate(zip(va, vb)):
            if len(x) != len(y):
                rep.fail('presentation', f'{name}[{i}]: {len(x)} vs {len(y)} values: {x[7:]} vs {y[7:]}')
                continue
            for j, (p, q) in enumerate(zip(x, y)):
                if p.get('kind') != q.get('kind'):
                    rep.fail('presentation', f'{name}[{i}][{j}] kind {p.get("kind")} vs {q.get("kind")}')
                elif isinstance(q.get('value'), list):
                    if len(p['value']) != len(q['value']) or not all(close(s, t, tol) for s, t in zip(p['value'], q['value'])):
                        rep.fail('presentation', f'{name}[{i}][{j}] {p["value"]} vs {q["value"]}')
                elif isinstance(q.get('value'), float):
                    if not close(p['value'], q['value'], tol):
                        rep.fail('presentation', f'{name}[{i}][{j}] {p["value"]} vs {q["value"]}')
                elif p.get('value') != q.get('value'):
                    rep.fail('presentation', f'{name}[{i}][{j}] {p.get("value")!r} vs {q.get("value")!r}')


def compare_brep(ours, ref, rep):
    for field in ('bodies', 'regions', 'shells', 'faces', 'loops', 'coedges', 'edges',
                  'vertices', 'points', 'surfaces', 'curves', 'configurations'):
        na, nb = len(ours.get(field, [])), len(ref.get(field, []))
        if na != nb:
            rep.fail('brep', f'{field}: {na} vs {nb}')
    for field in ('surfaces', 'curves'):
        ka = collections.Counter(x['geometry']['kind'] for x in ours.get(field, []))
        kb = collections.Counter(x['geometry']['kind'] for x in ref.get(field, []))
        if ka != kb:
            rep.fail('brep', f'{field} kinds: {dict(ka)} vs {dict(kb)}')
    ca = collections.Counter(json.dumps(f.get('color'), sort_keys=True) for f in ours.get('faces', []))
    cb = collections.Counter(json.dumps(f.get('color'), sort_keys=True) for f in ref.get('faces', []))
    if ca != cb:
        rep.fail('brep', f'face colours: {dict(ca)} vs {dict(cb)}')
    act_a = [(c.get('name'), len(c.get('bodies', []))) for c in ours.get('configurations', []) if c.get('active')]
    act_b = [(c.get('name'), len(c.get('bodies', []))) for c in ref.get('configurations', []) if c.get('active')]
    if act_a != act_b:
        rep.fail('brep', f'active configuration {act_a} vs {act_b}')


def compare_assembly(ours, ref, rep):
    oa, ob = ours.get('occurrences', []), ref.get('occurrences', [])
    if len(oa) != len(ob):
        rep.fail('assembly', f'occurrences: {len(oa)} vs {len(ob)}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('ours')
    p.add_argument('ref')
    p.add_argument('--only', default='tessellations,ownership,appearance,presentation,brep,assembly')
    p.add_argument('--tol', type=float, default=1e-9)
    args = p.parse_args()
    ours = load(args.ours)['model']
    ref = load(args.ref)['model']
    areas = set(args.only.split(','))
    rep = Report()
    if 'tessellations' in areas or 'assembly' in areas:
        compare_tessellations(ours, ref, rep, args.tol)
    if 'ownership' in areas:
        compare_ownership(ours, ref, rep)
    if 'appearance' in areas:
        compare_appearance(ours, ref, rep)
    if 'presentation' in areas:
        compare_presentation(ours, ref, rep, args.tol)
    if 'brep' in areas:
        compare_brep(ours, ref, rep)
    if 'assembly' in areas:
        compare_assembly(ours, ref, rep)
    if areas != {'tessellations', 'ownership', 'appearance', 'presentation', 'brep', 'assembly'}:
        for area in list(rep.failures):
            if area not in areas:
                del rep.failures[area]
    sys.exit(rep.print())


if __name__ == '__main__':
    main()
