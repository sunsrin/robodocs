// Copyright (c) 2026 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

// Topic configuration
const fuelTopicName = "/FuelVisualizer/Fuel";
const robotPoseTopicName = "/FuelVisualizer/Robot";
const isRedTopicName = "/FuelVisualizer/IsRed";

// Field Configuration
const FIELD_CONFIG = {
  image: "field.png",
  imageWidth: 2982,
  imageHeight: 1425,
  topLeft: [63, 15],
  bottomRight: [2918, 1409],
  widthMeters: 16.540988,
  heightMeters: 8.0689958,
};

// Dimensions
const FUEL_DIAMETER_INCHES = 5.91;
const FUEL_RADIUS_METERS = (FUEL_DIAMETER_INCHES * 0.0254) / 2;
const ROBOT_SIZE_METERS = 0.75;

let fuelPositions = [];
let robotPose = null; // {x, y, rot}
let isRed = false;

const fieldImage = new Image();
fieldImage.src = FIELD_CONFIG.image;
fieldImage.onload = () => requestAnimationFrame(render);

// ***** NETWORKTABLES *****
const ntClient = new NT4_Client(
  window.location.hostname,
  "FuelVisualizer",
  () => {},
  () => {},
  (topic, _, value) => {
    if (topic.name === fuelTopicName && Array.isArray(value)) {
      fuelPositions = [];
      for (let i = 0; i < value.length; i += 2) {
        if (i + 1 < value.length)
          fuelPositions.push({ x: value[i], y: value[i + 1] });
      }
    } else if (
      topic.name === robotPoseTopicName &&
      Array.isArray(value) &&
      value.length >= 3
    ) {
      robotPose = { x: value[0], y: value[1], rot: value[2] };
    } else if (topic.name == isRedTopicName && typeof value == "boolean") {
      isRed = value;
    }
    requestAnimationFrame(render);
  },
  () => {
    document.body.classList.remove("disconnected");
    requestAnimationFrame(render);
  },
  () => {
    document.body.classList.add("disconnected");
  },
);

window.addEventListener("load", () => {
  ntClient.subscribe([fuelTopicName], false, false, 1 / 20);
  ntClient.subscribe(
    [robotPoseTopicName, isRedTopicName],
    false,
    false,
    1 / 60,
  );
  ntClient.connect();
  window.addEventListener("resize", () => requestAnimationFrame(render));
});

/** Helper to transform points based on robot rotation. */
function transformPx(center, rotation, offset) {
  const cos = Math.cos(rotation);
  const sin = Math.sin(rotation);
  return [
    center[0] + offset[0] * cos - offset[1] * sin,
    center[1] - offset[0] * sin - offset[1] * cos,
  ];
}

function render() {
  const canvas = document.querySelector(".field-canvas");
  if (!canvas) return;
  const context = canvas.getContext("2d");
  const devicePixelRatio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  canvas.width = width * devicePixelRatio;
  canvas.height = height * devicePixelRatio;
  context.clearRect(0, 0, canvas.width, canvas.height);

  const scale = Math.min(
    canvas.width / FIELD_CONFIG.imageWidth,
    canvas.height / FIELD_CONFIG.imageHeight,
  );
  const drawWidth = FIELD_CONFIG.imageWidth * scale;
  const drawHeight = FIELD_CONFIG.imageHeight * scale;
  const offsetX = (canvas.width - drawWidth) / 2;
  const offsetY = (canvas.height - drawHeight) / 2;

  if (fieldImage.complete && fieldImage.naturalWidth > 0) {
    context.drawImage(fieldImage, offsetX, offsetY, drawWidth, drawHeight);
  }

  const playableWidthPx = FIELD_CONFIG.bottomRight[0] - FIELD_CONFIG.topLeft[0];
  const playableHeightPx =
    FIELD_CONFIG.bottomRight[1] - FIELD_CONFIG.topLeft[1];
  const pixelsPerMeterX = playableWidthPx / FIELD_CONFIG.widthMeters;
  const pixelsPerMeterY = playableHeightPx / FIELD_CONFIG.heightMeters;
  const avgPixelsPerMeter = (pixelsPerMeterX + pixelsPerMeterY) / 2;

  const toCanvas = (x, y) => {
    const imgX = FIELD_CONFIG.topLeft[0] + x * pixelsPerMeterX;
    const imgY = FIELD_CONFIG.bottomRight[1] - y * pixelsPerMeterY;
    return [offsetX + imgX * scale, offsetY + imgY * scale];
  };

  // Draw fuel
  const fuelRadiusCanvasPx = FUEL_RADIUS_METERS * avgPixelsPerMeter * scale;
  fuelPositions.forEach((pos) => {
    const [cx, cy] = toCanvas(pos.x, pos.y);
    context.beginPath();
    context.arc(cx, cy, fuelRadiusCanvasPx, 0, 2 * Math.PI);
    context.fillStyle = "yellow";
    context.fill();
  });

  // Draw robot
  if (robotPose) {
    const [rx, ry] = toCanvas(robotPose.x, robotPose.y);
    const rotation = robotPose.rot;
    const sizePx = ROBOT_SIZE_METERS * avgPixelsPerMeter * scale;

    context.save();
    context.lineCap = "round";
    context.lineJoin = "round";

    // Bumper
    context.fillStyle = "#222";
    context.strokeStyle = isRed ? "red" : "cyan";
    context.lineWidth = 6 * devicePixelRatio;

    const corners = [
      [0.5, 0.5],
      [0.5, -0.5],
      [-0.5, -0.5],
      [-0.5, 0.5],
    ].map((c) =>
      transformPx([rx, ry], rotation, [c[0] * sizePx, c[1] * sizePx]),
    );

    context.beginPath();
    context.moveTo(...corners[0]);
    corners.slice(1).forEach((c) => context.lineTo(...c));
    context.closePath();
    context.fill();
    context.stroke();

    // Directional arrow
    context.strokeStyle = "white";
    context.lineWidth = 3 * devicePixelRatio;
    const arrowBack = transformPx([rx, ry], rotation, [sizePx * -0.3, 0]);
    const arrowFront = transformPx([rx, ry], rotation, [sizePx * 0.3, 0]);
    const arrowLeft = transformPx([rx, ry], rotation, [
      sizePx * 0.15,
      sizePx * 0.15,
    ]);
    const arrowRight = transformPx([rx, ry], rotation, [
      sizePx * 0.15,
      sizePx * -0.15,
    ]);

    context.beginPath();
    context.moveTo(...arrowBack);
    context.lineTo(...arrowFront);
    context.lineTo(...arrowLeft);
    context.moveTo(...arrowFront);
    context.lineTo(...arrowRight);
    context.stroke();

    context.restore();
  }
}
