const BASE = import.meta.env.VITE_API_URL ?? "http://localhost:5000";

async function req(method, url, body, isForm = false) {
  const init = { method, headers: {} };
  if (body) {
    if (isForm) {
      init.body = body;
    } else {
      init.body = JSON.stringify(body);
      init.headers["Content-Type"] = "application/json";
    }
  }
  const res = await fetch(BASE + url, init);
  if (!res.ok) throw new Error(await res.text());
  return res.status === 204 ? null : res.json();
}

export const MapsAPI = {
  list: () => req("GET", "/maps"),
  upload: (name, pgm, yaml) => {
    const f = new FormData();
    f.append("name", name);
    f.append("pgm", pgm);
    f.append("yaml", yaml);
    return req("POST", "/maps", f, true);
  },
  delete: name => req("DELETE", `/maps/${name}`)
};

export const PointsAPI = {
  list: mapId => req("GET", `/points${mapId ? `?map_id=${mapId}` : ""}`),
  create: p => req("POST", "/points", p),
  delete: id => req("DELETE", `/points/${id}`)
};

export const MissionsAPI = {
  list: mapId => req("GET", `/missions${mapId ? `?map_id=${mapId}` : ""}`),
  create: m => req("POST", "/missions", m),
  delete: id => req("DELETE", `/missions/${id}`)
};
