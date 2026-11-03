import json, sys, time, urllib.request, urllib.parse, os
D = sys.argv[1]; lst = sys.argv[2]
out = os.path.join(D, 'verify')
UA = {'User-Agent': 'litreview-script/1.0'}
def get(url):
    for i in range(3):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=40) as r:
                return json.loads(r.read().decode('utf-8'))
        except urllib.error.HTTPError as e:
            if e.code == 404: return {'_404': True}
            time.sleep(3)
        except Exception: time.sleep(3)
    return None
for doi in [l.strip() for l in open(lst) if l.strip() and not l.startswith('#')]:
    fn = os.path.join(out, doi.replace('/', '_').replace('(', '').replace(')', '') + '.json')
    if os.path.exists(fn): j = json.load(open(fn))
    else:
        j = get('https://api.crossref.org/works/' + urllib.parse.quote(doi, safe='/'))
        if j and not j.get('_404'): json.dump(j, open(fn, 'w'))
    if not j or j.get('_404'): print(f'NOTFOUND {doi}'); continue
    m = j['message']
    au = '; '.join((a.get('family') or a.get('name') or '?') for a in (m.get('author') or m.get('editor') or []))
    yr = ((m.get('issued') or {}).get('date-parts') or [[None]])[0][0]
    ct = (m.get('container-title') or [''])[0]
    print(f'OK {doi} | {yr} | {(m.get("title") or [""])[0][:110]} | {au[:80]} | {ct[:50]} | {m.get("volume","")}({m.get("issue","")}):{m.get("page","")} | {m.get("type")}')
