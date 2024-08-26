import { IK, IKChain, IKJoint, IKBallConstraint } from 'three-ik';
import * as THREE from 'three';

export class IKSolver {
  constructor(model) {
    this.model = model;
    this.ik = new IK();
    this.chains = {};
  }

  createChain(chainName, jointNames) {
    const chain = new IKChain();
    const joints = jointNames.map((name, index) => {
      const bone = new THREE.Bone();
      bone.name = name;
      bone.position.set(0, index * 0.1, 0);
      const joint = new IKJoint(bone);
      return joint;
    });
  
    joints.forEach((joint, i) => {
      chain.add(joint, { constraints: [new IKBallConstraint(180)] });
    });
  
    this.ik.add(chain);
    this.chains[chainName] = chain;
  }

  solve(chainName, currentPositions, targetPosition) {
    console.log('Entering solve method');
    console.log('Chain name:', chainName);
    console.log('Current positions:', currentPositions);
    console.log('Target position:', targetPosition);

    const chain = this.chains[chainName];
    if (!chain) {
      console.error(`Chain ${chainName} not found`);
      return null;
    }

    console.log('Chain found');

    // Set current positions
    let prevPosition = null;
    chain.joints.forEach((joint, i) => {
      let position = currentPositions[i].clone();
      if (prevPosition && position.distanceTo(prevPosition) < 0.01) {
        // Add a significant offset if positions are too close
        const offset = new THREE.Vector3(0, 0.01, 0);
        position.add(offset);
      }
      joint.position.copy(position);
      prevPosition = position;
    });

    // Set target
    chain.target.position.copy(targetPosition);

    // Solve IK
    console.log('Before solving:', chain.joints.map(j => j.position.toArray()));
    this.ik.solve();
    console.log('After solving:', chain.joints.map(j => j.position.toArray()));


    // Return new joint rotations
    return chain.joints.map(joint => joint.rotation.toArray());
  }
}