import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import   load_mujoco        from '../dist/mujoco_wasm.js';

import { LogStdLayer } from './utils/LogStdLayer.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

const defaultRewardConfig = {
  originalPos: {
    expCoefficient: 2,
    subtractionFactor: 0.05,
    maxDiffNorm: 0.5,
  },
  orientationReward: {
    expCoefficient: 1,
  },
  weights: {
    ctrlCost: 0.1,
    originalPos: 4,
    orientation: 1,
    velocity: 1.25,
    height: 0,
  },
};

// Set up Emscripten's Virtual File System
var initialScene = "humanoid.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/" + initialScene, await(await fetch("./examples/scenes/" + initialScene)).text());

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML
    this.model      = new mujoco.Model("/working/" + initialScene);
    this.state      = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, useModel: true, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0, ikEnabled: false};
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio( window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    this.state = null;
    this.actuatorNames = [];
    this.actuatorRanges = [];
    this.loadPPOModel();

    this.ikEndEffectors = ['hand_left', 'hand_right'];
    this.ikTargets = {
      'hand_left': new THREE.Vector3(),
      'hand_right': new THREE.Vector3()
    };
    this.ikJointChains = {
      'hand_left': ['shoulder1_left', 'shoulder2_left', 'elbow_left'],
      'hand_right': ['shoulder1_right', 'shoulder2_right', 'elbow_right']
    };

    this.estimateControlForPositionDiff = this.estimateControlForPositionDiff.bind(this);
    this.ikControlRecords = [];

    this.bodyNameToId = this.createBodyNameToIdMap();
    this.actuatorNameToId = this.createActuatorNameToIdMap();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);
    document.addEventListener('keydown', this.handleKeyPress.bind(this));

    this.initialQpos = new Float64Array(this.simulation.qpos);
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =  
      await loadSceneFromURL(mujoco, initialScene, this);


    this.gui = new GUI();
    setupGUI(this);

    // this.initializeActuators();
  }

  // does this ordering align with the ordering of the model output?
  initializeActuators() {
    const textDecoder = new TextDecoder();
    for (let i = 0; i < this.model.nu; i++) {
      if (!this.model.actuator_ctrllimited[i]) { continue; }
      let name = textDecoder.decode(
        this.model.names.subarray(
          this.model.name_actuatoradr[i])).split('\0')[0];
      this.actuatorNames.push(name);
      this.actuatorRanges.push([
        this.model.actuator_ctrlrange[2 * i],
        this.model.actuator_ctrlrange[2 * i + 1]
      ]);
    }
  }

  handleKeyPress(event) {
    const key = event.key.toLowerCase();
    const stepSize = 0.1;

    switch (key) {
      case 'q':
        this.moveActuator('hip_y', stepSize);
        break;
      case 'a':
        this.moveActuator('hip_y_', -stepSize);
        break;
      case 'w':
        this.moveActuator('hip_', stepSize);
        break;
      case 's':
        this.moveActuator('hip_', -stepSize);
        break;
      case 'e':
        this.moveActuator('knee_', stepSize);
        break;
      case 'd':
        this.moveActuator('knee_', -stepSize);
        break;
      case 'r':
        this.moveActuator('abdomen_y', stepSize);
        break;
      case 'f':
        this.moveActuator('abdomen_y', -stepSize);
        break;
      case 't':
        this.moveActuator('ankle_', stepSize);
        break;
      case 'g':
        this.moveActuator('ankle_', -stepSize);
        break;
      case 'y':
        this.moveActuator('shoulder1_', stepSize);
        this.moveActuator('shoulder2_', stepSize);
        break;
      case 'h':
        this.moveActuator('shoulder1_', -stepSize);
        this.moveActuator('shoulder2_', -stepSize);
        break;
      case 'u':
        this.moveActuator('elbow_', stepSize);
        break;
      case 'j':
        this.moveActuator('elbow_', -stepSize);
        break;
    }
  }

  solveIK(endEffectorId) {
    if (!this.params.ikEnabled) return;
  
    const bodyName = this.getBodyNameById(endEffectorId);
    if (!this.ikEndEffectors.includes(bodyName)) {
      console.error(`Body "${bodyName}" is not an end effector`);
      return;
    }
  
    const endEffectorBody = this.bodies[endEffectorId];
    if (!endEffectorBody) {
      console.error(`End effector body "${bodyName}" not found`);
      return;
    }
  
    const targetPosition = this.ikTargets[bodyName];
    const jointChain = this.ikJointChains[bodyName];
    const maxIterations = 10;
    const epsilon = 0.01;
    const timeStep = this.model.getOptions().timestep;
  
    let iterationRecord = {
      targetPosition: targetPosition.toArray(),
      joints: {}
    };
  
    console.log(`IK Target for ${bodyName}: ${targetPosition.toArray()}`);
  
    for (let iteration = 0; iteration < maxIterations; iteration++) {
      const currentPosition = new THREE.Vector3();
      getPosition(this.simulation.xpos, endEffectorId, currentPosition);
      const error = targetPosition.clone().sub(currentPosition);
  
      if (error.length() < epsilon) {
        console.log(`Target for ${bodyName} reached, stopping IK`);
        break;
      }
  
      for (const jointName of jointChain) {
        const jointId = this.getJointIdByName(jointName);
        if (jointId === -1) {
          console.warn(`Joint ${jointName} not found`);
          continue;
        }
  
        const jacobian = this.calculateJacobian(jointId, endEffectorId);
  
        if (jacobian.lengthSq() < 1e-10) {
          console.warn(`Near-zero Jacobian for joint ${jointName}, skipping`);
          continue;
        }
  
        const delta = error.dot(jacobian) / jacobian.lengthSq();
  
        // Limit the maximum change in joint position
        const maxDelta = 0.1;
        const clampedDelta = Math.max(-maxDelta, Math.min(maxDelta, delta));
  
        const jointQposAdr = this.model.jnt_qposadr[jointId];
        this.simulation.qpos[jointQposAdr] += clampedDelta;
  
        // Estimate control for this joint
        const estimatedControl = this.estimateControlForPositionDiff(jointId, clampedDelta, timeStep, endEffectorId);
  
        // Record the joint information
        if (!iterationRecord.joints[jointName]) {
          iterationRecord.joints[jointName] = [];
        }
        iterationRecord.joints[jointName].push({
          positionChange: clampedDelta,
          estimatedControl: estimatedControl
        });
  
        console.log(`    New joint position for ${jointName}: ${this.simulation.qpos[jointQposAdr]}`);
      }
  
      this.simulation.forward();
    }
  
    // Add the iteration record to the overall IK control records
    this.ikControlRecords.push(iterationRecord);
  }

  calculateJacobian(jointId, endEffectorId) {
    const epsilon = 0.001;
    const jacobian = new THREE.Vector3();
  
    const endEffectorBodyId = endEffectorId;
    if (endEffectorBodyId === undefined) {
      console.error(`End effector body not found`);
      return jacobian;
    }
  
    // Get the joint's qpos address
    const jointQposAdr = this.model.jnt_qposadr[jointId];
    const originalValue = this.simulation.qpos[jointQposAdr];
  
    const originalPos = new THREE.Vector3();
    getPosition(this.simulation.xpos, endEffectorBodyId, originalPos);
  
    // Positive perturbation
    this.simulation.qpos[jointQposAdr] = originalValue + epsilon;
    this.simulation.forward();
    const posPos = new THREE.Vector3();
    getPosition(this.simulation.xpos, endEffectorBodyId, posPos);
  
    // Negative perturbation
    this.simulation.qpos[jointQposAdr] = originalValue - epsilon;
    this.simulation.forward();
    const posNeg = new THREE.Vector3();
    getPosition(this.simulation.xpos, endEffectorBodyId, posNeg);
  
    // Calculate Jacobian column using central difference
    jacobian.x = (posPos.x - posNeg.x) / (2 * epsilon);
    jacobian.y = (posPos.y - posNeg.y) / (2 * epsilon);
    jacobian.z = (posPos.z - posNeg.z) / (2 * epsilon);
  
    // Restore original state
    this.simulation.qpos[jointQposAdr] = originalValue;
    this.simulation.forward();
  
    return jacobian;
  }
  
  estimateControlForPositionDiff(jointIndex, positionDiff, timeStep, endEffectorId) {
    const jointAddress = this.model.jnt_qposadr[jointIndex];
  
    // Get inverse mass and damping for the joint
    let invMass = this.model.dof_invweight0 ? this.model.dof_invweight0[jointAddress] : 1 / timeStep;
    let damping = this.model.dof_damping ? this.model.dof_damping[jointAddress] : 0.1;
  
    // Estimate velocity required to achieve position difference
    const estimatedVelocity = positionDiff / timeStep;
  
    // Calculate force required to overcome damping and achieve velocity
    const dampingForce = damping * estimatedVelocity;
    const accelerationForce = invMass * estimatedVelocity;
  
    console.log(`End effector ID: ${endEffectorId}`);
    console.log("velocity", estimatedVelocity);
    console.log("dampingForce", dampingForce);
    console.log("accelerationForce", accelerationForce);
  
    // Total force is the sum of damping and acceleration forces
    const totalForce = dampingForce + accelerationForce;
  
    // Convert force to control input (assuming control input is proportional to force)
    const controlScale = 1.0; // Adjust this based on your model's characteristics
    const estimatedControl = totalForce * controlScale;
  
    return estimatedControl;
  }

  /* MAPPINGs */

  getJointIdByName(name) {
    for (let i = 0; i < this.model.njnt; i++) {
      const jointName = this.getJointName(i);
      if (jointName === name) {
        return i;
      }
    }
    return -1;
  }
  
  getJointName(jointId) {
    const textDecoder = new TextDecoder();
    return textDecoder.decode(
      this.model.names.subarray(
        this.model.name_jntadr[jointId]
      )
    ).split('\0')[0];
  }

  createBodyNameToIdMap() {
    const map = {};
    const textDecoder = new TextDecoder();
    for (let i = 0; i < this.model.nbody; i++) {
      const name = textDecoder.decode(
        this.model.names.subarray(
          this.model.name_bodyadr[i]
        )
      ).split('\0')[0];
      map[name] = i;
    }
    return map;
  }

  getBodyNameById(bodyId) {
    const textDecoder = new TextDecoder();
    return textDecoder.decode(
      this.model.names.subarray(
        this.model.name_bodyadr[bodyId]
      )
    ).split('\0')[0];
  }

  getBodyIdByName(name) {
    return this.bodyNameToId[name];
  }

  createActuatorNameToIdMap() {
    const map = {};
    const textDecoder = new TextDecoder();
    for (let i = 0; i < this.model.nu; i++) {
      const name = textDecoder.decode(
        this.model.names.subarray(
          this.model.name_actuatoradr[i]
        )
      ).split('\0')[0];
      map[name] = i;
    }
    return map;
  }

  getActuatorIdByName(name) {
    return this.actuatorNameToId[name] !== undefined ? this.actuatorNameToId[name] : -1;
  }

  getActuatorNameForJoint(jointName) {
    // This is a simplification. You might need to adjust this based on your naming conventions
    for (const actuatorName of this.actuatorNames) {
      if (actuatorName.includes(jointName)) {
        return actuatorName;
      }
    }
    return null;
  }


  /* Actuators/Model control*/

  moveActuator(prefix, amount) {
    for (let i = 0; i < this.actuatorNames.length; i++) {
      if (this.actuatorNames[i].startsWith(prefix)) {
        let currentValue = this.simulation.ctrl[i];
        let [min, max] = this.actuatorRanges[i];
        let newValue = Math.max(min, Math.min(max, currentValue + amount));
        this.simulation.ctrl[i] = newValue;
        this.params[this.actuatorNames[i]] = newValue;
      }
    }
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }


  applyControls(timestep) {
    for (let i = 0; i < this.actuatorNames.length; i++) {
      const actuatorName = this.actuatorNames[i];
      const jointIndex = this.model.actuator_trnid[2 * i];
      const jointAddress = this.model.jnt_qposadr[jointIndex];

      // Get current position and velocity
      const currentPosition = this.simulation.qpos[jointAddress];
      const currentVelocity = this.simulation.qvel[jointAddress];

      // Get desired position from control input
      const desiredPosition = this.params[actuatorName];

      // PD control
      const kp = 100; // Proportional gain
      const kd = 10;  // Derivative gain

      const positionError = desiredPosition - currentPosition;
      const velocityError = -currentVelocity; // Assuming desired velocity is 0

      const control = kp * positionError + kd * velocityError;

      // Apply the control force
      this.simulation.qfrc_applied[jointAddress] += control;
      console.log(this.simulation.qfrc_applied[jointAddress]);
    }
  }


  // should re-get pawel-diff
  async loadPPOModel() {
    let modelPath;

    switch (this.params.scene) {
      case 'humanoid.xml':
        modelPath = 'models/humanoid_stand_4_frams_noise_1e-4/model.json';
        this.getObservation = () => this.getObservationSkeleton(0, 10, 6);
        break;
      case 'blank':
        modelPath = 'models/cvals+2_frames/model.json';
        break;
      case 'brax_humanoid.xml':
        modelPath = 'models/brax_humanoid_cvalless_just_stand/model.json';
        this.getObservation = () => this.getObservationSkeleton(0, -1, -1);
        break;
      case 'brax_humanoidstandup.xml':
        modelPath = 'models/brax_humanoid_standup/model.json';
        this.getObservation = () => this.getObservationSkeleton(0, 20, 12);
        break;
      case 'dora/dora2.xml':
        modelPath = 'models/dora/model.json';
        this.getObservation = () => this.getObservationSkeleton(0, 100, 72);
        break;
      default:
        throw new Error(`Unknown Tensorflow.js model for XML path: ${this.params.scene}`);
    }

    // Load the model with custom objects
    this.ppo_model = await tf.loadLayersModel(modelPath);
  }

  getObservationSkeleton(qpos_slice, cinert_slice, cvel_slice) {
    const qpos = this.simulation.qpos.slice(qpos_slice);
    const qvel = this.simulation.qvel;
    const cinert = cinert_slice !== -1 ? this.simulation.cinert.slice(cinert_slice) : [];
    const cvel = cvel_slice !== -1 ? this.simulation.cvel.slice(cvel_slice) : [];
    const qfrc_actuator = this.simulation.qfrc_actuator;
  
    // console.log('qpos length:', qpos.length);
    // console.log('qvel length:', qvel.length);
    // console.log('cinert length:', cinert.length);
    // console.log('cvel length:', cvel.length);
    // console.log('qfrc_actuator length:', qfrc_actuator.length);
  
    const obsComponents = [
      ...qpos,
      ...qvel,
      ...cinert,
      ...cvel,
      ...qfrc_actuator
    ];
  
    return obsComponents;
  }

  computeReward(rewardConfig = defaultRewardConfig) {
  
    if (this.state == null || this.state.qpos == null) {
      this.state = this.simulation;
      return 0
    }

    let state = this.state;
    let nextState = this.simulation;
    let action = this.simulation.ctrl;

    // Helper functions
    const norm = (vector) => Math.sqrt(vector.reduce((sum, val) => sum + val * val, 0));
    const clip = (value, min, max) => Math.min(Math.max(value, min), max);
    const dot = (a, b) => a.reduce((sum, val, i) => sum + val * b[i], 0);

  
    // MAINTAINING ORIGINAL POSITION REWARD
    const qpos0Diff = this.initialQpos.map((q, i) => q - state.qpos[i]);
    const qpos0DiffNorm = norm(qpos0Diff);

    const originalPos = Math.exp(
      -rewardConfig.originalPos.expCoefficient * qpos0DiffNorm
    ) - rewardConfig.originalPos.subtractionFactor * 
      clip(qpos0DiffNorm, 0, rewardConfig.originalPos.maxDiffNorm);

    // ORIENTATION REWARD
    const torsoIndex = 1;
    const torsoOrientation = state.xmat.slice(torsoIndex * 9, torsoIndex * 9 + 9); // (N, 3, 3) flattened
    const gravityVector = [0, 0, -1];  // Assuming z-up coordinate system
    const projectedGravity = [
      dot(torsoOrientation.slice(0, 3), gravityVector),
      dot(torsoOrientation.slice(3, 6), gravityVector),
      dot(torsoOrientation.slice(6, 9), gravityVector)
    ];

    const orientation = Math.exp(-norm(projectedGravity.slice(0, 2)) * rewardConfig.orientationReward.expCoefficient);

    // CONTROL COST
    const ctrlCost = -Math.exp(-norm(action));


    // VELOCITY REWARD
    const centerOfMassIndex = 1;
    const xpos = state.subtree_com[centerOfMassIndex * 3 + 0]; // (N, 3) flattened
    const nextXpos = nextState.subtree_com[centerOfMassIndex * 3 + 0];
    const velocity = (nextXpos - xpos) / state.dt;
  
    // HEIGHT REWARD
    const height = state.qpos[2];
  
    // Calculate weighted rewards
    const ctrlCostWeighted = rewardConfig.weights.ctrlCost * ctrlCost;
    const originalPosWeighted = rewardConfig.weights.originalPos * originalPos;
    const velocityWeighted = rewardConfig.weights.velocity * velocity;
    const orientationWeighted = rewardConfig.weights.orientation * orientation;
    const heightWeighted = rewardConfig.weights.height * height;
  
    // Calculate total reward
    const totalReward = ctrlCostWeighted +
      originalPosWeighted +
      orientationWeighted +
      heightWeighted;
  
    // Calculate proportions
    const ctrlCostProp = ctrlCostWeighted / totalReward;
    const originalPosProp = originalPosWeighted / totalReward;
    const velocityProp = velocityWeighted / totalReward;
    const orientationProp = orientationWeighted / totalReward;
    // const heightProp = heightWeighted / totalReward;
  
    // Log proportions (equivalent to jax.debug.print)
    console.log(
      `Reward proportions: total_reward: ${totalReward}, `
      + `ctrl_cost: ${ctrlCostProp}, `
      + `original_pos: ${originalPosProp}, `
      + `orientation: ${orientationProp}`
      // + `height: ${heightProp}`
    );
  
    this.state = nextState;

    return totalReward;
  }


  // render loop
  render(timeMS) {
    this.controls.update();

    if (!this.params["paused"]) {

      // // reset to original state before paused
      // if (this.pausedState) {
      //   this.simulation.qpos.set(this.pausedState.qpos);
      //   this.simulation.ctrl.set(this.pausedState.ctrl);
      //   this.pausedState = null;
      // }

      // Update beforePauseQpos when unpaused
      if (!this.beforePauseQpos || this.beforePauseQpos.length !== this.simulation.qpos.length) {
        this.beforePauseQpos = new Float64Array(this.simulation.qpos);
      } else {
        this.beforePauseQpos.set(this.simulation.qpos);
      }

      if (this.ppo_model && this.params.useModel) { 
        const observationArray = this.getObservation();
        const inputTensor = tf.tensor2d([observationArray]);

        this.computeReward();

        try {
          const prediction = this.ppo_model.predict(inputTensor);
          
          if (!Array.isArray(prediction) || prediction.length !== 3) {
            console.error('Unexpected prediction output:', prediction);
            return;
          }
      
          const [actorMean, logStd, criticValue] = prediction;
      
          // console.log('Actor Mean:', actorMean.arraySync());
          // console.log('Log Std:', logStd.arraySync());
          // console.log('Critic Value:', criticValue.arraySync());

          // Update the critic value in the GUI
          this.params.criticValue = criticValue.dataSync()[0];
        
          // Use tf.tidy to automatically dispose of intermediate tensors
          tf.tidy(() => {
            const stdDev = tf.exp(logStd);
            const noise = tf.randomNormal(actorMean.shape);
            const actions = actorMean.add(stdDev.mul(noise));
      
            const actionData = actions.dataSync();
      
            // Update actuator controls
            for (let i = 0; i < actionData.length; i++) {
              if (i < this.simulation.ctrl.length) {
                let action = actionData[i];
                
                // Clip action to [-1, 1] range
                let clippedAction = Math.max(-1, Math.min(1, action));
      
                // Scale action to actuator range
                let [min, max] = this.actuatorRanges[i];
                let newValue = min + (clippedAction + 1) * (max - min) / 2;

                // Add noise to the new value
                if (this.params["ctrlnoisestd"] > 0.0) {
                  newValue += this.params["ctrlnoisestd"] * standardNormal(); // Add Gaussian noise
                }
                  
                this.simulation.ctrl[i] = newValue;
                this.params[this.actuatorNames[i]] = newValue;
              } else {
                console.error('Model output index out of bounds:', i);
              }
            }
          });
      
          // Dispose of tensors
          inputTensor.dispose();
          actorMean.dispose();
          logStd.dispose();
          criticValue.dispose();
        } catch (error) {
          console.error('Error during model prediction:', error);
        }
      }

      let timestep = this.model.getOptions().timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {

        // updates states from dragging
        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0 && !this.params["useModel"]) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params[this.actuatorNames[i]] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.simulation.qfrc_applied.length; i++) { this.simulation.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.simulation.xpos , b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        this.simulation.step();

        this.mujoco_time += timestep * 1000.0;
      }
    } else if (this.params["paused"]) {

      // // store the state on pause to restore to later
      // if (!this.pausedState) {
      //   this.pausedState = {
      //     qpos: new Float64Array(this.simulation.qpos),
      //     ctrl: new Float64Array(this.simulation.ctrl)
      //   };
      // }

      // Inverse kinematics
      this.dragStateManager.update(); // Update the world-space force origin
      if (this.params.ikEnabled) { 
        let dragged = this.dragStateManager.physicsObject;

      
        if (dragged && dragged.bodyID) {
          const bodyName = this.getBodyNameById(dragged.bodyID);
          if (!this.ikEndEffectors.includes(bodyName)) {
            console.error(`Body "${bodyName}" is not an end effector`);
          } else {
            this.ikTargets[bodyName].copy(this.dragStateManager.currentWorld);
            this.solveIK([dragged.bodyID]);
          }
        }
      } else {
        // updates states from dragging
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          let b = dragged.bodyID;
          getPosition  (this.simulation.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
          getQuaternion(this.simulation.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

          let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
            .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
          if (this.model.body_mocapid[b] >= 0) {
            // Set the root body's mocap position...
            console.log("Trying to move mocap body", b);
            let addr = this.model.body_mocapid[b] * 3;
            let pos  = this.simulation.mocap_pos;
            pos[addr+0] += offset.x;
            pos[addr+1] += offset.y;
            pos[addr+2] += offset.z;
          } else {
            // Set the root body's position directly...
            let root = this.model.body_rootid[b];
            let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
            let pos  = this.simulation.qpos;
            pos[addr+0] += offset.x;
            pos[addr+1] += offset.y;
            pos[addr+2] += offset.z;
          }
        }
      }

      // Showing differences in position while paused (IK is better)
      // const beforePauseQpos = this.beforePauseQpos || new Float64Array(this.simulation.qpos);
      // const timestep = this.model.getOptions().timestep;

      // // Apply changes based on control inputs
      // for (let i = 0; i < this.actuatorNames.length; i++) {
      //   const actuatorName = this.actuatorNames[i];
      //   const jointIndex = this.model.actuator_trnid[2 * i];
      //   const jointAddress = this.model.jnt_qposadr[jointIndex];

      //   // figure out how this physics simulator works
      //   let inv_mass = 1 / timestep; // Default value
      //   let damping = 0.1; // Default value

      //   if (this.model.dof_invweight0 && this.model.dof_invweight0[jointAddress] !== undefined) {
      //     inv_mass = this.model.dof_invweight0[jointAddress];
      //   }

      //   if (this.model.dof_damping && this.model.dof_damping[jointAddress] !== undefined) {
      //     damping = this.model.dof_damping[jointAddress];
      //   }
        
      //   // Calculate torque from control input
      //   const torqueDiff = this.params[actuatorName];
        
        
      //   // Calculate joint velocity
      //   let jointVelocity = 0.01;
      //   if (beforePauseQpos[jointAddress]) {
      //     const positionDiff = this.simulation.qpos[jointAddress] - beforePauseQpos[jointAddress];
      //     jointVelocity = positionDiff / timestep;
          
      //     // Handle potential NaN or Infinity
      //     if (!isFinite(jointVelocity)) {
      //       jointVelocity = 0;
      //       console.warn(`Invalid velocity calculated for joint ${jointAddress}. Using 0.`);
      //     }
      //   }

      //   // Calculate target deviation
      //   const targetDeviation = (torqueDiff * inv_mass - damping * jointVelocity) * timestep;
        
      //   // Calculate current deviation
      //   const currentDeviation = this.simulation.qpos[jointAddress] - beforePauseQpos[jointAddress];
        
      //   // Gradually move towards target deviation
      //   const step = 0.1; // Adjust for faster or slower response
      //   const newDeviation = currentDeviation + (targetDeviation - currentDeviation) * step;
        
      //   // Apply the new deviation
      //   this.simulation.qpos[jointAddress] = beforePauseQpos[jointAddress] + newDeviation;
      // }

      this.simulation.forward();
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.simulation.xpos , b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update tendon transforms.
    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] -1 ; w++) {
          let tendonStart = getPosition(this.simulation.wrap_xpos, w    , new THREE.Vector3());
          let tendonEnd   = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
          let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);
          
          let validStart = tendonStart.length() > 0.01;
          let validEnd   = tendonEnd  .length() > 0.01;

          if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validEnd  ) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validStart && validEnd) {
            mat.compose(tendonAvg, new THREE.Quaternion().setFromUnitVectors(
              new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres  .count = numWraps > 0 ? numWraps + 1: 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
    }

    // Render!
    this.renderer.render( this.scene, this.camera );
  }
}

let demo = new MuJoCoDemo();
await demo.init();
