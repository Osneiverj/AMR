const base = __API_URL__;

export async function login(username, password) {
  const res = await fetch(`${base}/auth/login`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams({ username, password })
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}
