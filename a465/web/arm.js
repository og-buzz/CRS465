// The A465's shape, modelled by hand from the drawings in the A465 Robot Arm User
// Guide (Fig. 2 elevation / plan views, Fig. 27 ready position). Link lengths come
// from the server's model; the 514 mm shoulder width is the drawing's dimension.
// Other proportions are measured off the drawings, so this is a likeness, not CAD.
// Each joint's local frame has Z along the link when the arm is straight up.
import * as THREE from 'three';

const BODY = new THREE.MeshStandardMaterial({ color: 0xe8e5de, roughness: 0.45, metalness: 0.05 });
const DARK = new THREE.MeshStandardMaterial({ color: 0x23272e, roughness: 0.55, metalness: 0.2 });
const MARK = new THREE.MeshStandardMaterial({ color: 0xf2a900, roughness: 0.5 });
export const HOUSING_COLOR = 0x23272e;

function mesh(geometry, material) {
  const m = new THREE.Mesh(geometry, material);
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}

function box(size, centre, material) {
  const m = mesh(new THREE.BoxGeometry(...size), material);
  m.position.set(...centre);
  return m;
}

// A box whose top face is a different size from its bottom face.
function taperedBox([bottomX, bottomY], [topX, topY], z0, z1, material) {
  const geometry = new THREE.BoxGeometry(1, 1, 1);
  const p = geometry.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const top = p.getZ(i) > 0;
    p.setXYZ(i, p.getX(i) * (top ? topX : bottomX), p.getY(i) * (top ? topY : bottomY), top ? z1 : z0);
  }
  geometry.computeVertexNormals();
  return mesh(geometry, material);
}

// Cylinders (or cones, with a second radius) along each axis, between two coordinates.
function tubeY(radius, y0, y1, material, endRadius = radius) {
  const geometry = new THREE.CylinderGeometry(endRadius, radius, y1 - y0, 48);
  geometry.translate(0, (y0 + y1) / 2, 0);
  return mesh(geometry, material);
}

function tubeZ(radius, z0, z1, material, endRadius = radius) {
  const geometry = new THREE.CylinderGeometry(endRadius, radius, z1 - z0, 48);
  geometry.rotateX(Math.PI / 2);
  geometry.translate(0, 0, (z0 + z1) / 2);
  return mesh(geometry, material);
}

function tubeX(radius, x0, x1, material) {
  const geometry = new THREE.CylinderGeometry(radius, radius, x1 - x0, 24);
  geometry.rotateZ(Math.PI / 2);
  geometry.translate((x0 + x1) / 2, 0, 0);
  return mesh(geometry, material);
}

// A plate lying in the XZ plane, `thickness` deep in Y and centred on `y`, from an outline in (x, z).
function plate(outline, thickness, y, material) {
  const bevel = 3;
  const geometry = new THREE.ExtrudeGeometry(outline, {
    depth: thickness - 2 * bevel, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments: 32,
  });
  geometry.translate(0, 0, -(thickness - 2 * bevel) / 2);
  geometry.rotateX(Math.PI / 2);                   // outline's second axis becomes Z, depth becomes Y
  geometry.translate(0, y, 0);
  return mesh(geometry, material);
}

// Two circles joined by their outer tangents: radius r0 at z = 0, radius r1 at z = length.
function taperedLink(r0, r1, length) {
  const lean = Math.asin((r0 - r1) / length);
  const outline = new THREE.Shape();
  outline.moveTo(r0 * Math.cos(lean), r0 * Math.sin(lean));
  outline.lineTo(r1 * Math.cos(lean), length + r1 * Math.sin(lean));
  outline.absarc(0, length, r1, lean, Math.PI - lean, false);
  outline.lineTo(-r0 * Math.cos(lean), r0 * Math.sin(lean));
  outline.absarc(0, 0, r0, Math.PI - lean, 2 * Math.PI + lean, false);
  return outline;
}

// A strap of the given half-width from z0 up to a rounded end centred on z1.
function roundedStrap(halfWidth, z0, z1) {
  const outline = new THREE.Shape();
  outline.moveTo(halfWidth, z0);
  outline.lineTo(halfWidth, z1);
  outline.absarc(0, z1, halfWidth, 0, Math.PI, false);
  outline.lineTo(-halfWidth, z0);
  outline.closePath();
  return outline;
}

// The track: a beam with two rails and end stops, lying along X below the arm's base.
// Returns the fixed rail and the carriage plate that travels with the arm.
export function buildTrack(travel) {
  const rail = new THREE.Group();
  const length = travel + 520;
  rail.add(box([length, 220, 100], [travel / 2, 0, -90], DARK));
  for (const y of [-80, 80]) rail.add(box([length, 22, 12], [travel / 2, y, -34], BODY));
  for (const x of [-235, travel + 235]) rail.add(box([40, 220, 50], [x, 0, -15], MARK));
  const carriage = box([300, 264, 28], [0, 0, -14], DARK);
  return { rail, carriage, floorZ: -140 };
}

export function buildArm(model) {
  const root = new THREE.Group();
  const pivots = [];
  // One material per joint for the part that turns red when that joint is at its limit.
  const housing = model.joints.map(() => new THREE.MeshStandardMaterial({ color: HOUSING_COLOR, roughness: 0.55, metalness: 0.2 }));

  let parent = root;
  for (const joint of model.joints) {
    const mount = new THREE.Group();
    mount.position.set(...joint.offset);
    parent.add(mount);
    const pivot = new THREE.Group();
    mount.add(pivot);
    pivots.push(pivot);
    parent = pivot;
  }
  const [waist, shoulder, elbow, wristRotate, wristPitch, toolRoll] = pivots;
  const shoulderHeight = model.joints[1].offset[2];
  const upperArm = model.joints[2].offset[2];
  const forearm = model.joints[4].offset[2];
  const hand = model.flange_offset[2];

  // Fixed base: plinth, tapered pedestal, and the two arm connectors at the back.
  root.add(box([204, 204, 46], [0, 0, 23], DARK));
  root.add(taperedBox([200, 200], [176, 176], 46, 176, BODY));
  root.add(tubeX(17, -122, -98, DARK).translateZ(72), tubeX(17, -122, -98, DARK).translateZ(116));

  // Waist: seam ring, turret, and the cone-shaped shoulder motor housings (514 mm tip to tip).
  waist.add(box([180, 180, 8], [0, 0, 180], housing[0]));
  waist.add(taperedBox([176, 176], [150, 200], 184, shoulderHeight - 40, BODY));
  for (const side of [-1, 1]) {
    const cheek = tubeY(92, 54, 100, BODY);
    const cone = tubeY(92, 100, 257, housing[1], 44);
    for (const part of [cheek, cone]) {
      part.scale.y = side;                         // mirror for the other side
      part.position.z = shoulderHeight;
      waist.add(part);
    }
  }

  // Upper arm: a tapered slab between the shoulder and elbow.
  shoulder.add(plate(taperedLink(70, 56, upperArm), 100, 0, BODY));

  // Elbow hub with its motor can, and the fixed lower forearm that houses the wrist-rotate drive.
  elbow.add(tubeY(60, -75, 75, housing[2]), tubeY(42, 75, 112, housing[2]));
  elbow.add(box([96, 96, 100], [0, 0, 70], BODY));

  // Rotating forearm: seam ring, a taper, and the two-sided yoke that carries the wrist pitch axis.
  wristRotate.add(tubeZ(56, 120, 128, housing[3]));
  wristRotate.add(taperedBox([96, 96], [78, 118], 128, 200, BODY));
  for (const side of [-1, 1]) wristRotate.add(plate(roundedStrap(39, 200, forearm), 18, side * 50, BODY));

  // Wrist pitch hub between the yoke plates, then the hand.
  wristPitch.add(tubeY(31, -41, 41, housing[4]));
  wristPitch.add(tubeZ(27, 0, 46, BODY));

  // Tool roll: neck, flange (its face is the tool point), a mark that shows roll, and a simple gripper.
  toolRoll.add(tubeZ(24, 46, hand - 12, BODY));
  toolRoll.add(tubeZ(31.5, hand - 12, hand, housing[5]));
  toolRoll.add(box([8, 8, 12], [31.5, 0, hand - 6], MARK));
  toolRoll.add(box([26, 92, 12], [0, 0, hand + 6], DARK));
  const fingers = [-1, 1].map((side) => {
    const finger = box([16, 10, 72], [0, 0, hand + 48], DARK);
    finger.userData.side = side;
    toolRoll.add(finger);
    return finger;
  });

  const triad = new THREE.AxesHelper(130);          // tool frame: X out of the flange
  triad.position.z = hand;
  triad.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(
    new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 1, 0), new THREE.Vector3(-1, 0, 0)));
  triad.material.depthTest = false;
  triad.renderOrder = 10;
  toolRoll.add(triad);

  return { root, pivots, housing, fingers, toolMark: MARK };
}
