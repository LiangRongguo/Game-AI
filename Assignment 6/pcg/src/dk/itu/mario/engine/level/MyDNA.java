package dk.itu.mario.engine.level;

import java.util.Random;
import java.util.*;

//Make any new member variables and functions you deem necessary.
//Make new constructors if necessary
//You must implement mutate() and crossover()


public class MyDNA extends DNA
{
	
	public int numGenes = 0; //number of genes

	// Return a new DNA that differs from this one in a small way.
	// Do not change this DNA by side effect; copy it, change the copy, and return the copy.
	public MyDNA mutate ()
	{
		MyDNA copy = new MyDNA();
		//YOUR CODE GOES BELOW HERE

		// get current chromosome
		String chromosome = this.getChromosome();
		// System.out.println("current chromosome " + chromosome);

		// Random class
		Random random = new Random();

		// index
		int index_selected = random.nextInt(chromosome.length());
		// new char
		char new_char = (char) ('a' + random.nextInt(26));

		// copied chromosome with one new char
		char[] newChromosome = chromosome.toCharArray();
		newChromosome[index_selected] = new_char;

		// set new chromosome to new copied DNA
		copy.setChromosome(String.valueOf(newChromosome));

		//YOUR CODE GOES ABOVE HERE
		return copy;
	}
	
	// Do not change this DNA by side effect
	public ArrayList<MyDNA> crossover (MyDNA mate)
	{
		ArrayList<MyDNA> offspring = new ArrayList<MyDNA>();
		//YOUR CODE GOES BELOW HERE

		// get chromosome
		String chromosome1 = this.getChromosome();
		String chromosome2 = mate.getChromosome();
		int len = chromosome1.length();
		
		/*
		System.out.println("Crossover:");
		System.out.println(chromosome1);
		System.out.println(chromosome2);
		*/

		// two result DNAs
		MyDNA newDNA1 = new MyDNA();
		MyDNA newDNA2 = new MyDNA();

		// Random class
		Random random = new Random();
		// random break point
		int breakPoint = random.nextInt(len);

		// crossover
		String newChromosome1 = chromosome1.substring(0, breakPoint) + chromosome2.substring(breakPoint, len);
		String newChromosome2 = chromosome2.substring(0, breakPoint) + chromosome1.substring(breakPoint, len);

		newDNA1.setChromosome(newChromosome1);
		newDNA2.setChromosome(newChromosome2);

		offspring.add(newDNA1);
		offspring.add(newDNA2);
		//YOUR CODE GOES ABOVE HERE
		return offspring;
	}
	
	// Optional, modify this function if you use a means of calculating fitness other than using the fitness member variable.
	// Return 0 if this object has the same fitness as other.
	// Return -1 if this object has lower fitness than other.
	// Return +1 if this objet has greater fitness than other.
	public int compareTo(MyDNA other)
	{
		int result = super.compareTo(other);
		//YOUR CODE GOES BELOW HERE
		
		//YOUR CODE GOES ABOVE HERE
		return result;
	}
	
	
	// For debugging purposes (optional)
	public String toString ()
	{
		String s = super.toString();
		//YOUR CODE GOES BELOW HERE
		
		//YOUR CODE GOES ABOVE HERE
		return s;
	}
	
	public void setNumGenes (int n)
	{
		this.numGenes = n;
	}

}

