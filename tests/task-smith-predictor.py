from agimus_sot.sot import TaskSmithPredictor
from math import pi
import matplotlib.pyplot as plt

dt = 0.01
real_delay = 10
est_delay = 8

from agimus_sot.control.controllers import secondOrderOpenLoop

freq = 4. # Hz
damp = .5

i_model = 0
def makeModel():
  global i_model
  i_model += 1
  return secondOrderOpenLoop("model_" + str(i_model), 2*pi*freq, damp, dt, (0.,))

def checkModel(model, N, ref):
  outputs = list()
  start = model.reference.time
  for i in range(start+1,start+1+N):
    model.reference.time = i
    model.reference.value = ref
    model.output.recompute(i)
    outputs.append(model.output.value)
  return outputs

def checkModel2(N=100):
  model = makeModel()
  outputs = checkModel(model, N, (1.,))
  plt.plot(outputs)
  plt.show()

def checkModel3 (N, ref, delay, noise_sigma=0.):
  import random

  model = makeModel()
  gain = 6.
  outputs = list()
  states = list()
  errors = list()
  measurements = list()

  start = model.reference.time
  init_cur = 0.
  #cur = model.output.value[0]
  cur = init_cur
  state = cur

  states.append(state)
  outputs.append(cur)

  for i in range(start+1,start+1+N):
    vel = gain*(ref-cur)
    #vel = gain*(ref-state)
    errors.append(vel)
    state += vel * dt
    model.reference.time = i
    model.reference.value = (state,)
    model.output.recompute(i)

    if i >= start+1+delay:
    	cur = outputs[i-start-1-delay]
    else:
    	cur = init_cur

    cur += random.gauss(0, noise_sigma)

    states.append(state)
    outputs.append(model.output.value[0])
    measurements.append(cur)

  plt.plot(states, 'r', label="motor ref")
  plt.plot(outputs, 'g', label="motor pos (real)")
  plt.plot(errors, 'b', label="error")
  plt.plot(measurements, 'y', label="motor pos (measured)")
  plt.legend()
  plt.show()

def checkSmithPredictor (N, ref, delay, est_delay=None, noise_sigma=0.):
  import random
  if est_delay is None:
    est_delay = delay

  model = makeModel()
  gain = 6.
  outputs = list()
  states = list()
  errors = list()
  measurements = list()

  tsp = TaskSmithPredictor("")
  tsp.initialize(dt, est_delay)
  from dynamic_graph.sot.core.feature_1d import Feature1D
  feature = Feature1D("")
  tsp.add(feature.name)
  tsp.controlGain.value = gain

  start = model.reference.time
  init_cur = 0.
  #cur = model.output.value[0]
  cur = init_cur
  state = cur

  states.append(state)
  outputs.append(cur)

  for i in range(start+1,start+1+N):
    feature.error.value = (cur - ref,)
    feature.error.time = i
    tsp.task.recompute(i)
    vel = tsp.task.value[0]

    errors.append(vel)
    state += vel * dt
    model.reference.time = i
    model.reference.value = (state,)
    model.output.recompute(i)

    if i >= start+1+delay:
    	cur = outputs[i-start-1-delay]
    else:
    	cur = init_cur

    cur += random.gauss(0, noise_sigma)

    states.append(state)
    outputs.append(model.output.value[0])
    measurements.append(cur)

  plt.plot(states, 'r', label="motor ref")
  plt.plot(outputs, 'g', label="motor pos (real)")
  plt.plot(errors, 'b', label="error")
  plt.plot(measurements, 'y', label="motor pos (measured)")
  plt.legend()
  plt.show()

def compare(N, ref, delay, est_delay=None, noise_sigma=0.):
  plt.figure()
  plt.title ("Delayed system")
  checkModel3(N, ref, delay, noise_sigma=noise_sigma)
  plt.figure()
  plt.title ("Delayed system with Smith predictor")
  checkSmithPredictor(N, ref, delay, est_delay=est_delay, noise_sigma=noise_sigma)
