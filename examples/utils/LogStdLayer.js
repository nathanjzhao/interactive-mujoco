export class LogStdLayer extends tf.layers.Layer {
  constructor(config) {
    super(config);
    this.action_dim = config.action_dim; // this doesn't actually do anything lol
  }

  build(inputShape) {
    // If action_dim is not provided in the config, use the last dimension of the input shape
    if (this.action_dim === undefined) {
      this.action_dim = Array.isArray(inputShape) ? inputShape[inputShape.length - 1] : inputShape.slice(-1)[0];
    }
    
    this.log_std = this.addWeight(
      'log_std',
      [this.action_dim],
      'float32',
      tf.initializers.zeros(),
      true
    );
  }

  call(inputs, kwargs) {
    return tf.tidy(() => {
      let batchSize;
      if (inputs instanceof tf.Tensor) {
        batchSize = inputs.shape[0];
      } else {
        batchSize = inputs.length;
      }
      
      // Ensure log_std is a tensor
      let logStdTensor;
      if (this.log_std instanceof tf.Tensor) {
        logStdTensor = this.log_std;
      } else if (this.log_std.read && typeof this.log_std.read === 'function') {
        const logStdValues = this.log_std.read();
        if (logStdValues instanceof tf.Tensor) {
          logStdTensor = logStdValues;
        } else if (Array.isArray(logStdValues) && logStdValues.length === this.action_dim) {
          logStdTensor = tf.tensor1d(logStdValues);
        } else {
          console.error('Unexpected log_std values:', logStdValues);
          logStdTensor = tf.zeros([this.action_dim]);
        }
      } else {
        console.error('Unexpected log_std type:', this.log_std);
        logStdTensor = tf.zeros([this.action_dim]);
      }
      
      // Broadcast log_std to match the batch size
      const broadcastedLogStd = tf.broadcastTo(logStdTensor, [batchSize, this.action_dim]);
      
      return broadcastedLogStd;
    });
  }

  getConfig() {
    const baseConfig = super.getConfig();
    return {
      ...baseConfig,
      action_dim: this.action_dim,
    };
  }

  static className = 'LogStdLayer';
}

tf.serialization.registerClass(LogStdLayer);