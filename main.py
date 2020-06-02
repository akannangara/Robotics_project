import sys
from bulletworld import bulletworld
from dqn import Agent
import numpy as np
from utils import plotLearning


#python3 main.py [d, dqn, number_games, ]

n_games = 0

def main(argv):
	print("Robotics Leidenuniv")
	print("race car project")
	if len(argv) > 1 and argv[1] == "dqn":
		print("running dqn")
		run_dqn()
		n_games = argv[2]
	elif len(argv) > 1 and argv[1] == "dqn_train":
		print("running train")
		run_dqn_camera(1)
		n_games = argv[2]
	elif len(argv) > 1 and argv[1] == "dqn_test":
		print("Running test")
		run_dqn_camera(2)
	else:
		env = bulletworld(argv, 1)
		env.run()
		del env
	print("program has ended")

def run_train(test_train):
	env = bulletworld(["d"], test_train)
	nr_games = int(sys.argv[3])
	agent = Agent(gamma=0.99, epsilon=1.0, alpha=0.0005, input_dims=3,
				n_actions=11, mem_size=1000000, batch_size=2, epsilon_end=0.01)
	scores = []
	eps_history = []
	for i in range (nr_games):
		done = False
		score = 0
		observation = env.reset_state()
		counter = 0
		while not done:
			action = agent.choose_action(observation)
			observation_, reward, done, info = env.step_pureCamera(action)
			score+=reward
			agent.remember(observation, action, reward, observation_, done)
			observation = observation_
			agent.learn()
			counter += 1
		eps_history.append(agent.epsilon)
		scores.append(score)
		avg_score = np.mean(scores[max(0,i-100):(i+1)])
		print('episode ', i, ' score %.2f' % score, 'average score %.2f' % avg_score, 'counter %.2f' % counter)
		if i %10 == 0 and i > 0:
			agent.save_model()
	agent.save_model()
	filename='racecarLearner.png'
	x = [i+1 for i in range(nr_games)]
	plotLearning(x, scores, eps_history, filename)

def run_test(test_train):
	env = bulletworld(["d"], test_train)
	#define alpha as 0.005?
	agent = Agent(gamma=0.99, epsilon=1.0, alpha=0.0005, input_dims=3,
				n_actions=11, mem_size=1000000, batch_size=2, epsilon_end=0.01)
	agent.load_model()
	observation = env.car.getState()
	score = 0
	while True:
		action = agent.choose_action(observation)
		observation_, reward, done, info = env.step_pureCamera(action)
		score+=reward
		agent.remember(observation, action, reward, observation_, done)
		observation = observation_
		agent.learn()

def run_dqn_camera(test_train):
	if test_train == 1:
		run_train(test_train)
	elif test_train == 2:
		run_test(test_train)

def run_dqn():
	env = bulletworld(["d"], 0)
	nr_games = int(sys.argv[3])
	agent = Agent(gamma=0.99, epsilon=1.0, alpha=0.0005, input_dims=2,
				n_actions=11, mem_size=1000000, batch_size=2, epsilon_end=0.01)
	scores = []
	eps_history = []
	for i in range (nr_games):
		done = False
		score = 0
		observation = env.reset()
		counter = 0
		while not done:
			action = agent.choose_action(observation)
			observation_, reward, done, info = env.step(action)
			score+=reward
			agent.remember(observation, action, reward, observation_, done)
			observation = observation_
			agent.learn()
			counter+=1
		eps_history.append(agent.epsilon)
		scores.append(score)
		avg_score = np.mean(scores[max(0,i-100):(i+1)])
		print('episode ', i, ' score %.2f' % score, 'average score %.2f' % avg_score, 'counter: %.2f' % counter)
		if i %10 == 0 and i > 0:
			agent.save_model()
	filename='racecarLearner.png'
	x = [i+1 for i in range(n_games)]
	plotLearning(x, scores, eps_hisotry, filename)

if __name__=="__main__":
	main(sys.argv[1:])
