* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
/**
 * @file: LaserScannerDriver.h
 *
 * @brief: Driver for a Lidar, see here: https://en.wikipedia.org/wiki/Lidar
 *
 * @author: Marco Martinelli
 * @date: 5/12/2020
 * @contact: martinellim45@gmail.com
 *
 */ 

#ifndef LASERSCANNERDRIVER_H
#define LASERSCANNERDRIVER_H

#include <iostream>
#include <vector>

class LaserScannerDriver{
private:
  const int MAX_SIZE_OF_COLUMNS = 1801;
  const int BUFFER_DIM = 10; //numero di scansioni costante
  double **buffer; //array 2d ottenuto con double pointer
  double ang_resolution; //risoluzione angolare
  int n_of_columns; //numero di scansioni (ottenuto dalla risoluzione angolare)
  int head; //indice della prima scansione effettuata
  int tail; //indice dell'ultima scansione effettuata
  int size_of_columns; //numero di letture
  int current_size; //numero di scansioni nel buffer

public:
//costruttore
  LaserScannerDriver(double ang_res = 1);

//costruttore di copia
  LaserScannerDriver(const LaserScannerDriver &arg);

//costruttore di move
  LaserScannerDriver(LaserScannerDriver &&arg);

//assegnamento di copia
  LaserScannerDriver &operator=(const LaserScannerDriver &arg);

//assegnamento di move
  LaserScannerDriver &operator=(LaserScannerDriver &&arg);

//distruttore
  ~LaserScannerDriver();

/*La funzione new_scan che accetta un vector<double> contenente una nuova scansione
fornita dal sensore e la memorizza nel buffer (sovrascrivendo quella più vecchia se il
buffer è pieno) – questa funzione esegue anche il controllo di dimensione: se le letture
sono in numero minore del previsto, completa i dati mancanti a 0, se sono in numero
maggiore, li taglia;*/
  void new_scan(const std::vector<double> &v);

/*La funzione get_scan che fornisce in output (in maniera opportuna) un vector<double>
contenete la scansione più vecchia del sensore e la rimuove dal buffer;*/
  std::vector<double> get_scan();

/*La funzione clear_buffer che elimina (senza ritornarle) tutte le letture salvate;*/
  void clear_buffer();

/*La funzione get_distance che accetta un angolo (double) e ritorna il valore di distanza
(cioè la lettura) corrispondente a tale angolo (tenendo conto della risoluzione angolare)
nella scansione più recente acquisita dal sensore – tale scansione non è eliminata dal
buffer, e se l’angolo richiesto non è disponibile la funzione ritorna il valore di angolo più
vicino;*/
  const double get_distance(double ang);

//UTILITY
//Ritorna il numero di colonne
  const int get_n_of_columns() const;

//Ritorna la risoluzione angolare
  const double get_ang_resolution() const;

//Ritorna l'indice della scansione più vecchia
  const int get_head() const;

//Ritorna l'indice della scansione più recente
  const int get_tail() const;

//Ritorna la dimensione delle colonne
  const int get_size_of_columns() const;

//Ritorna l'ultima scansione inserita nel buffer senza rimuoverla
  const std::vector<double> get_last_scan() const;

//Verifica se il buffer è vuoto
  const bool isEmpty();

//Verifica se il buffer è pieno  
  const bool isFull();

//Ritorna il numero di scansioni presenti nel buffer attualmente
  const int get_current_size() const;
  
};

/*L’overloading dell’operator<< che stampa a schermo l’ultima scansione salvata (ma non la
rimuove dal buffer).*/
  std::ostream& operator<<(std::ostream& os, const LaserScannerDriver& arg);

//Stampa i dettagli del buffer passato come argomento
  void print_state(const LaserScannerDriver& lsd );  

#endif
