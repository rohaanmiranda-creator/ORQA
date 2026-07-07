// ORQA preview service worker — network-first passthrough.
// Presence enables PWA installability; the full offline cache ships with the
// complete dist-web build (all models vendored). Preview keeps it simple.
self.addEventListener('install', () => self.skipWaiting());
self.addEventListener('activate', (e) => e.waitUntil(self.clients.claim()));
self.addEventListener('fetch', () => {});   // default network behaviour
