// https://stackoverflow.com/a/30144587/13164753
export function interpolateColor(color1Hex: string, color2Hex: string, weight: number) {
    const color1 = [parseInt(color1Hex.slice(1, 3), 16), parseInt(color1Hex.slice(3, 5), 16), parseInt(color1Hex.slice(5, 7), 16)];
    const color2 = [parseInt(color2Hex.slice(1, 3), 16), parseInt(color2Hex.slice(3, 5), 16), parseInt(color2Hex.slice(5, 7), 16)];
    const w1 = 1.0 - weight;
    const w2 = weight;
    var rgb = [Math.round(color1[0] * w1 + color2[0] * w2),
        Math.round(color1[1] * w1 + color2[1] * w2),
        Math.round(color1[2] * w1 + color2[2] * w2)];
    return `#${rgb[0].toString(16).padStart(2, '0')}${rgb[1].toString(16).padStart(2, '0')}${rgb[2].toString(16).padStart(2, '0')}`;
}