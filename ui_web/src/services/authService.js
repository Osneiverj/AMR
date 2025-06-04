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

export async function getMe(token) {
  const res = await fetch(`${base}/auth/me`, {
    headers: { Authorization: `Bearer ${token}` }
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}

export async function register(data, token) {
  const res = await fetch(`${base}/auth/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      Authorization: `Bearer ${token}`
    },
    body: JSON.stringify(data)
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}

export async function changePassword(newPassword, token) {
  const res = await fetch(`${base}/auth/change-password`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      Authorization: `Bearer ${token}`
    },
    body: JSON.stringify({ new_password: newPassword })
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}
