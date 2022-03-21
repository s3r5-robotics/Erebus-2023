import fs from 'fs';
import { fileURLToPath } from 'url';

const PATHING_REGEX = /sys\.path\.append\((\n|)(.*)\)/gm;

const srcDir = new URL('../src/', import.meta.url);
const srcPath = fileURLToPath(srcDir);

const sources = fs.readdirSync(srcPath);

for (const sourceFile of sources) {
    const path = fileURLToPath(new URL(sourceFile, srcDir));

    const file = fs.readFileSync(path, { encoding: 'utf8' });

    fs.writeFileSync(path, file.replace(PATHING_REGEX, `sys.path.append(r"${srcPath}")`.replace(/\\/gm, '\\\\')));
}
