import { createContext, useContext, useState, useEffect } from 'react';
import { login as apiLogin, getMe } from './services/authService';

const AuthContext = createContext(null);

export function AuthProvider({ children }) {
  const [token, setToken] = useState(() => localStorage.getItem('token'));
  const [user, setUser] = useState(null);

  useEffect(() => {
    if (token) {
      getMe(token)
        .then(setUser)
        .catch(() => {
          setToken(null);
          setUser(null);
          localStorage.removeItem('token');
        });
    } else {
      setUser(null);
    }
  }, [token]);

  const login = async (username, password) => {
    const data = await apiLogin(username, password);
    setToken(data.access_token);
    localStorage.setItem('token', data.access_token);
    const me = await getMe(data.access_token);
    setUser(me);
  };

  const logout = () => {
    setToken(null);
    setUser(null);
    localStorage.removeItem('token');
  };

  return (
    <AuthContext.Provider value={{ token, user, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useAuth = () => useContext(AuthContext);
