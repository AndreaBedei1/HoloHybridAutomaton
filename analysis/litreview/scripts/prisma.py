import json, os, re, sys, glob
D = sys.argv[1]
def norm(t): return re.sub(r'[^a-z0-9]', '', (t or '').lower())[:60]
uniq = {}; perq = 0; per_topic = {}
for fn in sorted(glob.glob(os.path.join(D, 'raw', '*.json'))):
    tag = os.path.basename(fn)[:-5]; topic = re.match(r'(T\d+|X)', tag).group(1)
    d = json.load(open(fn)); recs = {}
    for p in ((d.get('oa') or {}).get('results') or []):
        recs[norm(p.get('title'))] = (p.get('doi') or '').replace('https://doi.org/', '').lower()
    for it in (((d.get('cr') or {}).get('message') or {}).get('items') or []):
        k = norm((it.get('title') or [''])[0]); recs.setdefault(k, it['DOI'].lower())
    for p in ((d.get('s2') or {}).get('data') or []):
        k = norm(p.get('title')); recs.setdefault(k, ((p.get('externalIds') or {}).get('DOI') or '').lower())
    recs.pop('', None)
    perq += len(recs); per_topic[topic] = per_topic.get(topic, 0) + len(recs)
    for k, v in recs.items(): uniq.setdefault(k, v)
dois = set(v for v in uniq.values() if v)
print('per-query records', perq, 'unique records', len(uniq), 'per topic', per_topic)
inc = [l.split() for l in open(os.path.join(D, 'scripts', 'included.txt')) if l.strip()]
found = [k for t, k, doi in inc if doi.lower() in dois]
print('included found by database queries:', len(found), 'of', len(inc))
print('not found:', [k for t, k, doi in inc if doi.lower() not in dois])
