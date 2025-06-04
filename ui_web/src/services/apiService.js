const base = __API_URL__;

function authHeaders() {
  const token = localStorage.getItem('token');
  return token ? { Authorization: `Bearer ${token}` } : {};
}

export async function get(path) {
  const res = await fetch(`${base}${path}`, { headers: authHeaders() });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}

export async function post(path, data) {
  const res = await fetch(`${base}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', ...authHeaders() },
    body: JSON.stringify(data)
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}
