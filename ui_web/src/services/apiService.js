const base = __API_URL__;

export async function get(path) {
  const res = await fetch(`${base}${path}`);
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}

export async function post(path, data) {
  const res = await fetch(`${base}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}
