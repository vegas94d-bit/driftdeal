const fs = require('fs');
const html = fs.readFileSync('index.html', 'utf8');
const m = html.match(/<script>([\s\S]*)<\/script>/);
if (!m) throw new Error('script not found');
fs.writeFileSync('_game.js', m[1]);
console.log('extracted chars:', m[1].length);
