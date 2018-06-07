#!/usr/bin/env python

import math
import random
"""
   Demo of a PathPatch object.
   """
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt   



# City Class to handle only one city
class City:
   def __init__(self, x, y):
      self.x = x
      self.y = y
      
   def getX(self):
      return self.x
  
   def getY(self):
      return self.y
   
   # calculate distance from current city to distenation city
   def distanceTo(self, city):
      xDistance = abs(self.getX() - city.getX())
      yDistance = abs(self.getY() - city.getY())
      distance = math.sqrt( (xDistance*xDistance) + (yDistance*yDistance) )
      return distance
   
   def __repr__(self):
      return str(self.getX()) + ", " + str(self.getY())
   
#_________________________________________________________________________________________




#PathHandler class to handle cities of specific solution path  
class PathHandler:
    
   # Holds our cities
   destinationCities = []
   
   # Adds a destination city
   def addCity(self, city):
      self.destinationCities.append(city)
   # Get a city
   def getCity(self, index):
      return self.destinationCities[index]
   # Get the number of destination cities
   def numberOfCities(self):
      return len(self.destinationCities)
#_______________________________________________________________

#Path class to handle the candidate path

class PPath:
   def __init__(self, PathHandler, path=None):
      self.PathHandler = PathHandler
      self.path = []
      self.fitness = 0.0
      self.distance = 0
      if path is not None:
         self.path = path
      else:
         for i in range(0, self.PathHandler.numberOfCities()):
            self.path.append(None)
   
   def __repr__(self):
      geneString = "(City#1=>"
      startCity = self.path[0]
      self.path.append(startCity)
      fig, ax = plt.subplots()
      Path = mpath.Path      
      for i in range(0, self.pathSize()):
         j = i +2
         if i==0 :
            path_data = [
               (Path.MOVETO,(self.getCity(i).getX(),self.getCity(i). getY()))  ]  
         path_data+=[(Path.LINETO, (self.getCity(i).getX(),self.getCity(i). getY()))]
         if i==self.pathSize() :
            path_data+=[(Path.STOP, (self.getCity(i).getX(),self.getCity(i). getY()))]

         geneString += str(self.getCity(i)) + ") ("+"City#"+str(j)+"=>"
      codes, verts = zip(*path_data)
      path = mpath.Path(verts, codes)
      patch = mpatches.PathPatch(path, facecolor='g', alpha=0.1)
      ax.add_patch(patch) 
      #plot control points and connecting lines
      x, y = zip(*path.vertices)
      line = ax.plot(x, y, 'go-')   
      ax.grid()
      ax.axis('equal')
      plt.show()      
      return geneString
   
    # Creates a random path
   def generateIndividual(self,startCity):
      for cityIndex in range(0, self.PathHandler.numberOfCities()):
         self.setCity(cityIndex, self.PathHandler.getCity(cityIndex))
      random.shuffle(self.path) 
   #force the path to start from given city
      temp = City(self.path[0].getX(),self.path[0].getY())
      for i in range(0,self.pathSize()):
         if startCity.getX() == self.path[i].getX() and  startCity.getY() == self.path[i].getY():
            self.path[i] = temp
            self.path[0] = startCity
            break
   
   #  Gets a city from the path
   def getCity(self, Position):
      return self.path[Position]
   
   # Sets a city in a certain position within a path
   def setCity(self, Position, city):
      self.path[Position] = city
      self.fitness = 0.0
      self.distance = 0
   
   # Gets the tours fitness
   def getFitness(self):
      if self.fitness == 0:
         self.fitness = 1/float(self.getDistance())
      return self.fitness
   
   # Gets the total distance of the path
   def getDistance(self):
      if self.distance == 0:
         pathDistance = 0
         for cityIndex in range(0, self.pathSize()):
            fromCity = self.getCity(cityIndex)
            destinationCity = None
            if cityIndex+1 < self.pathSize():
               destinationCity = self.getCity(cityIndex+1)
            else:
               destinationCity = self.getCity(0)
            pathDistance += fromCity.distanceTo(destinationCity)
         self.distance = pathDistance
      return self.distance
   
   # Get number of cities on our tour
   def pathSize(self):
      return len(self.path)
   
   # Check if the tour contains a city
   def containsCity(self, city):
      return city in self.path

#_____________________________________________________________________

#Population  a class that Manages a finite set of candidate paths

class Population:
   
   def __init__(self, PathHandler, populationSize, initialise,startPoint):
      self.paths = []
      self.startPoint = startPoint
      for i in range(0, populationSize):
         self.paths.append(None)
      
      if initialise:
         for i in range(0, populationSize):
            newPath = PPath(PathHandler)
            newPath.generateIndividual(self.startPoint)
            self.savePath(i, newPath)
   
   
   #  Save a Path
   def savePath(self, index, Path):
      self.paths[index] = Path
   
   # Get a Path from population
   def getPath(self, index):
      return self.paths[index]
 
   # Get the best Path in the population
   def getFittest(self):
      fittest = self.paths[0]
      for i in range(0, self.populationSize()):
         if fittest.getFitness() <= self.getPath(i).getFitness():
            fittest = self.getPath(i)
      return fittest
   
   # Get population size
   def populationSize(self):
      return len(self.paths)

#GeneticAlgorithm a class that handle the working of the genetic algorithm and evolve our population of solutions paths

class GeneticAlgorithm:
   def __init__(self, PathHandler,startPoint):
      self.PathHandler = PathHandler
      self.mutationRate = 0.020
      self.tournamentSize = 5
      self.elitism = True
      self.startPoint = startPoint
   
   # Evolves a population over one generation
   def evolvePopulation(self, pop):
      newPopulation = Population(self.PathHandler, pop.populationSize(), False,self.startPoint)
      # Keep our best individual if elitism is enabled
      elitismOffset = 0
      if self.elitism:
         newPopulation.savePath(0, pop.getFittest())
         elitismOffset = 1
      
      
      for i in range(elitismOffset, newPopulation.populationSize()):
         # Select parents
         parent1 = self.tournamentSelection(pop)
         parent2 = self.tournamentSelection(pop)
         # Crossover parents
         child = self.crossover(parent1, parent2)
         # Add child to new population
         newPopulation.savePath(i, child)
      
      
      for i in range(elitismOffset, newPopulation.populationSize()):
         self.mutate(newPopulation.getPath(i))
            
      return newPopulation
   
  #Applies crossover to a set of parents
   def crossover(self, parent1, parent2):
      #Create new child path
      child = PPath(self.PathHandler)
      child.setCity(0, self.startPoint)
      # Get start and end sub path positions for parent1's path
      startPos = int(random.random() * parent1.pathSize())
      endPos = int(random.random() * parent1.pathSize())
      # Loop and add the sub path from parent1 to our child
      for i in range(0, child.pathSize()):
         # If our start position is less than the end position
         if startPos < endPos and i > startPos and i < endPos and i != 0:
            child.setCity(i, parent1.getCity(i))
         # If our start position is larger
         elif startPos > endPos:
            if i != 0 and  not (i < startPos and i > endPos):
               child.setCity(i, parent1.getCity(i))
      
      # Loop through parent2's city path
      for i in range(0, parent2.pathSize()):
          # If child doesn't have the city add it
         if not child.containsCity(parent2.getCity(i)):
             # Loop to find a spare position in the child's path
            for j in range(0, child.pathSize()):
               # Spare position found, add city
               if child.getCity(j) == None:
                  child.setCity(j, parent2.getCity(i))
                  break
               
      
      return child
   
  # Mutate a path using swap mutation
   def mutate(self, path):
      # Loop through path cities
      for Pos1 in range(1, path.pathSize()):
           # Apply mutation rate
         Pos2 = int(path.pathSize() * random.random())
         if random.random() < self.mutationRate and Pos2 != 0:
            # Get a second random position in the path
            
            
            # Get the cities at target position in path
            city1 = path.getCity(Pos1)
            city2 = path.getCity(Pos2)
            # Swap them around
            path.setCity(Pos2, city1)
            path.setCity(Pos1, city2)
            
   # Selects candidate path for crossover
   def tournamentSelection(self, pop):
      # Create a tournament population
      tournament = Population(self.PathHandler, self.tournamentSize, False,None)
      
      # For each place in the tournament get a random candidate path and add it
      for i in range(0, self.tournamentSize):
         randomId = int(random.random() * pop.populationSize())
         tournament.savePath(i, pop.getPath(randomId))
         
      # Get the fittest path
      fittest = tournament.getFittest()
      return fittest



if __name__ == '__main__':
   
   PathHandler = PathHandler()
   
   # Create and add our cities
   city = City(60, 200)
   PathHandler.addCity(city)
   city2 = City(180, 200)
   PathHandler.addCity(city2)
   city3 = City(80, 180)
   PathHandler.addCity(city3)
   city4 = City(140, 180)
   PathHandler.addCity(city4)
   city5 = City(20, 160)
   PathHandler.addCity(city5)
   city6 = City(100, 160)
   PathHandler.addCity(city6)
   city7 = City(200, 160)
   PathHandler.addCity(city7)
   city8 = City(140, 140)
   PathHandler.addCity(city8)
   city9 = City(40, 120)
   PathHandler.addCity(city9)
   city10 = City(100, 120)
   PathHandler.addCity(city10)
   city11 = City(180, 100)
   PathHandler.addCity(city11)
   city12 = City(60, 80)
   PathHandler.addCity(city12)
   city13 = City(120, 80)
   PathHandler.addCity(city13)
   city14 = City(180, 60)
   PathHandler.addCity(city14)
   city15 = City(20, 40)
   PathHandler.addCity(city15)
   city16 = City(100, 40)
   PathHandler.addCity(city16)
   city17 = City(200, 40)
   PathHandler.addCity(city17)
   city18 = City(20, 20)
   PathHandler.addCity(city18)
   city19 = City(60, 20)
   PathHandler.addCity(city19)
   city20 = City(160, 20)
   PathHandler.addCity(city20)
   city21 = City(30, 40)
   PathHandler.addCity(city21)    
   city22 = City(130, 40)
   PathHandler.addCity(city22)
   city23 = City(110, 40)
   PathHandler.addCity(city23)
   city24 = City(50, 20)
   PathHandler.addCity(city24)
   city25 = City(90, 20)
   PathHandler.addCity(city25)
   city26 = City(170, 20)
   PathHandler.addCity(city26) 
   city27 = City(10, 40)
   PathHandler.addCity(city27)
   city28 = City(135, 40)
   PathHandler.addCity(city28)
   city29 = City(300, 40)
   PathHandler.addCity(city29)
   city30 = City(210, 30)
   PathHandler.addCity(city30)
    
   
   startCity = City(120,80)  #start city 
   
   # Initialize population
   pop = Population(PathHandler, 50, True,startCity);
   #print ("Initial distance: " + str(pop.getFittest().getDistance()))
   
   Algorithm = GeneticAlgorithm(PathHandler,startCity)
   pop = Algorithm.evolvePopulation(pop)
   best_path = PPath(PathHandler)
   best_path = pop.getFittest()
   gen = 0
   for i in range(0, 100):
      pop = Algorithm.evolvePopulation(pop)
      print("Generation#"+str(i)+" =>"+str(pop.getFittest().getDistance()))
      if (best_path.getDistance()>=pop.getFittest().getDistance()):
         best_path = pop.getFittest()
         gen = i
   
   # Print final results
   print ("Final distance: " + str(pop.getFittest().getDistance()))
   print ("Shortest Path for Travelling Sales Person->")
   print (pop.getFittest())
   print("the Optimal Path found in generation: "+str(gen)+" Path Distance : "+str(best_path.getDistance()))
  # print("TSP Path: "+str(best_path))
   
   
   ##########
   
      
      
   