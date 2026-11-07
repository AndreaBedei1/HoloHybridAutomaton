import json, os, re, sys, glob
D = sys.argv[1]
def norm(t): return re.sub(r'[^a-z0-9]', '', (t or '').lower())[:60]
recs = {}
for fn in glob.glob(os.path.join(D, 'raw', '*.json')):
    d = json.load(open(fn))
    items = [((p.get('title')), (p.get('doi') or '').replace('https://doi.org/', '').lower()) for p in ((d.get('oa') or {}).get('results') or [])]
    items += [((it.get('title') or [''])[0], it['DOI'].lower()) for it in (((d.get('cr') or {}).get('message') or {}).get('items') or [])]
    items += [(p.get('title'), ((p.get('externalIds') or {}).get('DOI') or '').lower()) for p in ((d.get('s2') or {}).get('data') or [])]
    for t, doi in items:
        k = norm(t)
        if not k: continue
        if k not in recs or (doi and not recs[k]): recs[k] = doi
print('unique', len(recs))
ver = set()
for fn in glob.glob(os.path.join(D, 'verify', '*.json')):
    m = json.load(open(fn))['message']; ver.add(m['DOI'].lower())
wrong = {'10.1109/taes.1981.309142', '10.1109/iros.2016.7759239'}
ver -= wrong
print('assessed works', len(ver))
dbd = set(recs.values())
inter = ver & dbd
print('assessed found in db stream', len(inter))
pre = ('10.48550/', '10.21203/', '10.20944/', '10.2139/ssrn', '10.32388/', '10.31237/')
th = ('10.25549/', '10.1575/', '10.5075/', '10.70675/', '10.31390/', '10.22215/', '10.37099/', '10.33915/', '10.4995/thesis', '10.26190/', '10.15760/', '10.32657/', '10.14288/', '10.7282/', '10.63028/', '10.18297/', '10.23860/', '10.48526/', '10.3990/', '10.5281/zenodo')
c = {'nodoi': 0, 'preprint': 0, 'thesis_or_repository': 0, 'other': 0}
for k, doi in recs.items():
    if doi in inter: continue
    if not doi: c['nodoi'] += 1
    elif doi.startswith(pre): c['preprint'] += 1
    elif doi.startswith(th): c['thesis_or_repository'] += 1
    else: c['other'] += 1
print(c)
inc = set(l.split()[2].lower() for l in open(os.path.join(D, 'scripts', 'included.txt')) if l.strip())
exc = sorted(ver - inc)
print('excluded at eligibility', len(exc))
for doi in exc:
    fn = [f for f in glob.glob(os.path.join(D, 'verify', '*.json'))]
for f in glob.glob(os.path.join(D, 'verify', '*.json')):
    m = json.load(open(f))['message']
    if m['DOI'].lower() in exc:
        au = (m.get('author') or [{}])[0].get('family', '?')
        y = ((m.get('issued') or {}).get('date-parts') or [[None]])[0][0]
        print(f"{m['DOI'].lower()}|{au}|{y}|{(m.get('title') or [''])[0][:85]}|{'DB' if m['DOI'].lower() in dbd else 'OM'}")
