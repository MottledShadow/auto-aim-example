const fs = require("fs");

function findQuotedValue(header, key) {
    const keyPos = header.indexOf(key);
    if (keyPos < 0) {
        throw new Error(`missing npy header key: ${key}`);
    }

    const colon = header.indexOf(":", keyPos);
    for (let i = colon + 1; i < header.length; ++i) {
        const code = header.charCodeAt(i);
        if (code !== 34 && code !== 39) {
            continue;
        }

        let end = i + 1;
        while (end < header.length && header.charCodeAt(end) !== code) {
            ++end;
        }
        return header.slice(i + 1, end);
    }

    throw new Error(`invalid npy header value: ${key}`);
}

function parseShape(header) {
    const keyPos = header.indexOf("shape");
    const open = header.indexOf("(", keyPos);
    const close = header.indexOf(")", open + 1);
    if (keyPos < 0 || open < 0 || close < 0) {
        throw new Error("invalid npy shape field");
    }

    return header
        .slice(open + 1, close)
        .split(",")
        .map((item) => item.trim())
        .filter(Boolean)
        .map(Number);
}

function loadNpy(path) {
    const buffer = fs.readFileSync(path);
    if (buffer.slice(0, 6).toString("latin1") !== "\x93NUMPY") {
        throw new Error(`not a npy file: ${path}`);
    }

    const major = buffer[6];
    let offset = 8;
    let headerLength = 0;
    if (major === 1) {
        headerLength = buffer.readUInt16LE(offset);
        offset += 2;
    } else {
        headerLength = buffer.readUInt32LE(offset);
        offset += 4;
    }

    const header = buffer.slice(offset, offset + headerLength).toString("latin1");
    offset += headerLength;

    if (!header.includes("False")) {
        throw new Error("fortran-order npy arrays are not supported");
    }

    const descr = findQuotedValue(header, "descr");
    const shape = parseShape(header);
    const rows = shape.length === 1 ? 1 : shape[0];
    const cols = shape.length === 1 ? shape[0] : shape[1];
    const elementSize = descr.endsWith("f8") ? 8 : 4;
    const data = [];

    for (let i = 0; i < rows * cols; ++i) {
        const dataOffset = offset + i * elementSize;
        data.push(elementSize === 8 ? buffer.readDoubleLE(dataOffset) : buffer.readFloatLE(dataOffset));
    }

    return { rows, cols, data };
}

function formatNumber(value) {
    return Number(value)
        .toPrecision(16)
        .replace(/(\.\d*?)0+$/, "$1")
        .replace(/\.$/, "");
}

function opencvMatrix(name, matrix) {
    return [
        `${name}: !!opencv-matrix`,
        `   rows: ${matrix.rows}`,
        `   cols: ${matrix.cols}`,
        "   dt: d",
        `   data: [ ${matrix.data.map(formatNumber).join(", ")} ]`,
    ].join("\n");
}

if (process.argv.length !== 5) {
    console.error("Usage: node tools/convert_calibration_npy.js camera_matrix.npy dist_coeffs.npy output.yml");
    process.exit(1);
}

const cameraMatrix = loadNpy(process.argv[2]);
const distCoeffs = loadNpy(process.argv[3]);
const yaml = [
    "%YAML:1.0",
    "---",
    opencvMatrix("camera_matrix", cameraMatrix),
    opencvMatrix("dist_coeffs", distCoeffs),
    "",
].join("\n");

fs.writeFileSync(process.argv[4], yaml);
