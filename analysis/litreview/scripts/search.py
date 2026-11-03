import json, sys, time, urllib.request, urllib.parse, os, re
D = sys.argv[1]
Q = json.load(open(os.path.join(D, 'scripts', 'queries.json')))
only = sys.argv[2:]
UA = {'User-Agent': 'litreview-script/1.0'}
def get(url, tries=3, base=4):
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=40) as r:
                return json.loads(r.read().decode('utf-8'))
        except Exception as e:
            time.sleep(base*(i+1))
    return None
def norm(t): return re.sub(r'[^a-z0-9]', '', (t or '').lower())[:60]
counts = {}
for topic, qs in Q.items():
    if only and topic not in only: continue
    counts[topic] = 0
    for qi, q in enumerate(qs):
        tag = f'{topic}q{qi+1}'
        fn = os.path.join(D, 'raw', tag + '.json')
        data = json.load(open(fn)) if os.path.exists(fn) else {'query': q, 'date': time.strftime('%Y-%m-%d'), 's2': None, 'cr': None, 'oa': None}
        if data.get('oa') is None:
            data['oa'] = get('https://api.openalex.org/works?' + urllib.parse.urlencode({'filter': 'title_and_abstract.search:' + q, 'per-page': 20, 'select': 'doi,title,publication_year,cited_by_count,primary_location,type'}))
        if data.get('cr') is None:
            data['cr'] = get('https://api.crossref.org/works?' + urllib.parse.urlencode({'query': q, 'rows': 10, 'select': 'DOI,title,container-title,issued,type,is-referenced-by-count'}))
        if data.get('s2') is None:
            data['s2'] = get('https://api.semanticscholar.org/graph/v1/paper/search?' + urllib.parse.urlencode({'query': q, 'limit': 20, 'fields': 'title,year,venue,externalIds,citationCount'}), tries=1, base=1)
        json.dump(data, open(fn, 'w'))
        recs = {}
        def add(src, cit, yr, t, ven, doi):
            k = norm(t)
            if not k: return
            if k in recs:
                if src not in recs[k][0]: recs[k][0] += src
                recs[k][1] = max(recs[k][1], cit or 0)
                if not recs[k][5] and doi: recs[k][5] = doi
            else: recs[k] = [src, cit or 0, yr, t, ven or '', (doi or '').lower()]
        for p in ((data['oa'] or {}).get('results') or []):
            src = ((p.get('primary_location') or {}).get('source') or {}) or {}
            add('O', p.get('cited_by_count'), p.get('publication_year'), p.get('title'), src.get('display_name'), (p.get('doi') or '').replace('https://doi.org/', ''))
        for it in (((data['cr'] or {}).get('message') or {}).get('items') or []):
            yr = ((it.get('issued') or {}).get('date-parts') or [[None]])[0][0]
            add('C', it.get('is-referenced-by-count'), yr, (it.get('title') or [''])[0], (it.get('container-title') or [''])[0], it['DOI'])
        for p in ((data['s2'] or {}).get('data') or []):
            add('S', p.get('citationCount'), p.get('year'), p.get('title'), p.get('venue'), (p.get('externalIds') or {}).get('DOI', ''))
        n = {k: (len(((data[k] or {}).get('results') or [])) if k=='oa' else len((((data[k] or {}).get('message') or {}).get('items') or [])) if k=='cr' else len(((data[k] or {}).get('data') or []))) for k in ('oa','cr','s2')}
        counts[topic] += len(recs)
        print(f'## {tag} "{q}" OA={n["oa"]} CR={n["cr"]} S2={n["s2"]} unique={len(recs)}')
        for r in sorted(recs.values(), key=lambda r: -r[1]):
            print(f'{r[1]} {r[2]}|{r[3][:78]}|{r[5]}')
print('COUNTS', json.dumps(counts))
