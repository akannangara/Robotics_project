This project was created for the Robotics course at Leiden University.
It is important to note that this project is not complete and we
were unable to achieve our goal.

To run the program you will need:
-Keras
-numpy
-math
-OpenCV
-PyBullet

To run the program such that a DQN is used to train the car
to follow a white cube around the basic circular track, run the
command:
	python main.py d dqn [number of games]

To train the car using a DQN such that it teachs itself to
drive around the circular track, run the command:
	python main.py d dqn_train [number of games]

To test the trained car using the trained DQN model on the oval
track, run the command:
	python main.py d dqn_test
