import lcm
import numpy as np
import tensorflow as tf
from Network import Network  # Replace with your actual generated LCM class
import joblib
from PredictionMsg import PredictionMsg
# Load each leg's model
model_leg_1 = tf.keras.models.load_model('/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/leg_1_model.h5')
model_leg_2 = tf.keras.models.load_model('/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/leg_2_model.h5')
model_leg_3 = tf.keras.models.load_model('/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/leg_3_model.h5')
model_leg_4 = tf.keras.models.load_model('/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/leg_4_model.h5')
scaler_X_constant1 = joblib.load('scaler_X_constant1.pkl')
scaler_X_velocities_time_seq1 = joblib.load('scaler_X_velocities_time_seq1.pkl')
scaler_y1 = joblib.load('scaler_y1.pkl')

scaler_X_constant2 = joblib.load('scaler_X_constant2.pkl')
scaler_X_velocities_time_seq2 = joblib.load('scaler_X_velocities_time_seq2.pkl')
scaler_y2 = joblib.load('scaler_y2.pkl')

scaler_X_constant3 = joblib.load('scaler_X_constant3.pkl')
scaler_X_velocities_time_seq3 = joblib.load('scaler_X_velocities_time_seq3.pkl')
scaler_y3 = joblib.load('scaler_y3.pkl')

scaler_X_constant4 = joblib.load('scaler_X_constant4.pkl')
scaler_X_velocities_time_seq4 = joblib.load('scaler_X_velocities_time_seq4.pkl')
scaler_y4 = joblib.load('scaler_y4.pkl')

class Handler:
    def __init__(self, lcm_instance):
        self.lcm = lcm_instance  # Initialize the lcm instance
        # Initialize the outputs dictionary
        self.outputs = {
            "leg_1": None,
            "leg_2": None,
            "leg_3": None,
            "leg_4": None
        }

    def handle_message(self, channel, data):
        msg = Network.decode(data)  # Decode the incoming message

        # Print received message details
        print(f"Received message on channel \"{channel}\":")
        print(f"  delta_steps   = {msg.delta}")
        print(f"  time          = {msg.timestep}")

        # Prepare input data for the models
        X_constant_sim = np.array([[msg.cycletime,msg.bezierheight,msg.delta, msg.runstairs ,msg.pitch_ascension, msg.offsety]])  # Adjust this based on your actual input features
        X_time_seq_sim = np.array([[msg.xvelocity, msg.zvelocity, msg.timestep], [0, 0, 0]])  # Simulated time sequence input
        
        print(f"input_notscaled = {X_constant_sim}")
        print(f"input_notscaled_vt = {X_time_seq_sim}")

        X_constant_scaled1 = scaler_X_constant1.transform(X_constant_sim)
        X_time_seq_scaled1 = scaler_X_velocities_time_seq1.transform(X_time_seq_sim).reshape(1, 2, 3)

        X_constant_scaled2 = scaler_X_constant2.transform(X_constant_sim)
        X_time_seq_scaled2 = scaler_X_velocities_time_seq2.transform(X_time_seq_sim).reshape(1, 2, 3)

        X_constant_scaled3 = scaler_X_constant3.transform(X_constant_sim)
        X_time_seq_scaled3 = scaler_X_velocities_time_seq3.transform(X_time_seq_sim).reshape(1, 2, 3)


        X_constant_scaled4 = scaler_X_constant4.transform(X_constant_sim)
        X_time_seq_scaled4 = scaler_X_velocities_time_seq4.transform(X_time_seq_sim).reshape(1, 2, 3)

        print(f"input_scaled1 = {X_constant_scaled1}")
        print(f"input_scaled_vt1 = {X_time_seq_scaled1}")

        # Make predictions for each leg model
        self.outputs1 = model_leg_1.predict([X_constant_scaled1, X_time_seq_scaled1])
        self.outputs2 = model_leg_2.predict([X_constant_scaled2, X_time_seq_scaled2])
        self.outputs3 = model_leg_3.predict([X_constant_scaled3, X_time_seq_scaled3])
        self.outputs4 = model_leg_4.predict([X_constant_scaled4, X_time_seq_scaled4])
        
        
        
        predictions_inverse1 = scaler_y1.inverse_transform(self.outputs1)
        predictions_inverse2 = scaler_y2.inverse_transform(self.outputs2)
        predictions_inverse3 = scaler_y3.inverse_transform(self.outputs3)
        predictions_inverse4 = scaler_y4.inverse_transform(self.outputs4)

        # Create a message with the predictions
        pred_msg = PredictionMsg()
        pred_msg.leg_1_pred = predictions_inverse1[0][0]
        pred_msg.leg_2_pred = predictions_inverse2[0][0]
        pred_msg.leg_3_pred = predictions_inverse3[0][0]
        pred_msg.leg_4_pred = predictions_inverse4[0][0]

        # Publish the message
        self.lcm.publish("prediction_channel", pred_msg.encode())
        print(f"Published predictions: {pred_msg.leg_1_pred}, {pred_msg.leg_2_pred}, {pred_msg.leg_3_pred}, {pred_msg.leg_4_pred}")


def main():
    # Initialize the LCM
    lc = lcm.LCM()
    if lc is None:
        print("LCM initialization failed.")
        return

    # Create a handler instance and pass the LCM instance
    handler = Handler(lc)

    # Subscribe to the channel
    lc.subscribe("inputdatas", handler.handle_message)

    # Start listening for messages
    while True:
        lc.handle()  # This blocks and waits for messages


if __name__ == "__main__":
    main()
