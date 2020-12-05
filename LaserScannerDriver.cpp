#include "LaserScannerDriver.h"
#include <cmath>

//costruttore
  LaserScannerDriver::LaserScannerDriver(double ang_res){
    if(ang_res<0.1){
      std::cout<<"! ANGULAR RESOLUTION UNDER THE MIN(=0.1) ACCEPTED !\n Setted to 0.1\n";
      ang_res = 0.1;
    }

    if(ang_res>1){
      std::cout<<"! ANGULAR RESOLUTION OVER THE MAX(=1) ACCEPTED !\n Setted to 1\n";
      ang_res = 1;
    }

    ang_resolution = ang_res;
    n_of_columns = BUFFER_DIM;
    head = 0;
    current_size = 0;
    tail = 0;
    size_of_columns = (180/ang_res) +1;
    if(size_of_columns > MAX_SIZE_OF_COLUMNS)
      size_of_columns = MAX_SIZE_OF_COLUMNS;

    buffer = new double*[n_of_columns]; //inizializza buffer
    for(int i=0; i<n_of_columns; i++)
      buffer[i] = nullptr; //inizializza le colonne del buffer
  }

//distruttore
  LaserScannerDriver::~LaserScannerDriver(){
    for(int i=0; i<n_of_columns; i++){
      delete[] buffer[i]; //dealloca le colonne del buffer
      buffer[i] = nullptr;
    }

    delete[] buffer; //dealloca il buffer
  }

//costruttore di copia
  LaserScannerDriver::LaserScannerDriver(const LaserScannerDriver &arg):
    ang_resolution{arg.ang_resolution}, head{arg.head}, tail{arg.tail}, size_of_columns{arg.size_of_columns}, current_size{arg.current_size}, n_of_columns{arg.n_of_columns}
    {
      buffer = new double*[arg.n_of_columns]; //alloca nuovo spazio per il buffer
      for(int i=0; i<arg.n_of_columns; i++){
        if(arg.buffer[i] != nullptr){
          buffer[i] = new double[size_of_columns]; //alloca nuovo spazio per la colonna
          for(int j=0; j<size_of_columns; j++)
            buffer[i][j] = arg.buffer[i][j]; //copia il contenuto della colonna
        }
      }
    }
    


//costruttore di move
  LaserScannerDriver::LaserScannerDriver(LaserScannerDriver &&arg):
    ang_resolution{arg.ang_resolution}, head{arg.head}, tail{arg.tail}, size_of_columns{arg.size_of_columns}, current_size{arg.current_size}, n_of_columns{arg.n_of_columns}
    {
      buffer = arg.buffer;

      arg.buffer = nullptr; //annullo buffer arg
      arg.n_of_columns = 0; 
      arg.ang_resolution = 0;
      arg.head = 0;
      arg.tail = 0;
      arg.size_of_columns = 0;
      arg.current_size = 0;
    }

//assegnamento di copia
  LaserScannerDriver& LaserScannerDriver::operator=(const LaserScannerDriver &arg){
    if(size_of_columns != arg.size_of_columns || get_ang_resolution() != arg.get_ang_resolution()){
      std::cout<<"! LIDAR INCOMPATIBLE ! \nstd::exception has been thrown\n"; //se i due buffer hanno ris. ang. diverse la copia è impossibile
      throw std::exception();
    }

    for(int i=0; i<n_of_columns; i++){
      delete[] buffer[i]; //dealloca colonne del buffer vecchio
      buffer[i] = nullptr; 
    }
    delete[] buffer; //dealloca buffer vecchio

    head = arg.head; //recupero variabili di esemplare
    tail = arg.tail;
    current_size = arg.current_size;
    ang_resolution = arg.ang_resolution;
    size_of_columns = arg.size_of_columns;
    double** new_buffer = new double*[n_of_columns]; //alloca nuovo spazio per il buffer

    for(int i=0; i<n_of_columns; i++){
      if(arg.buffer[i] != nullptr){
        new_buffer[i] = new double[size_of_columns]; //alloca nuovo spazio per la colonna
        for(int j=0; j<size_of_columns; j++)
          new_buffer[i][j] = arg.buffer[i][j]; //copia il contenuto della colonna
      }
    }

    buffer = new_buffer; //recupero puntatore
    n_of_columns = arg.n_of_columns; //recupero size
    size_of_columns = arg.size_of_columns; //recuper onumero di letture
    return *this; //ritorno self-reference
  }

//assegnamento di move 
  LaserScannerDriver& LaserScannerDriver::operator=(LaserScannerDriver &&arg){
    if(size_of_columns != arg.size_of_columns || get_ang_resolution() != arg.get_ang_resolution()){
      std::cout<<"! LIDAR INCOMPATIBLE ! \n Angular resolution setted to "<<arg.get_ang_resolution()<<"\n";
      ang_resolution = arg.ang_resolution; //ho reputato corretto consentire l'operazione anche se le ris. ang. sono diverse per rimanere fedele alla definizione di move
    }

    for(int i=0; i<n_of_columns; i++){
      delete[] buffer[i]; //dealloco colonne del buffer vecchio
      buffer[i] = nullptr;
    }
    delete[] buffer; //dealloco buffer vecchio

    buffer = arg.buffer; //recupero puntatore
    n_of_columns = arg.n_of_columns; 
    head = arg.head; 
    tail = arg.tail; 
    current_size = arg.current_size; 

    arg.buffer = nullptr; //annullo buffer arg
    arg.n_of_columns = 0;
    arg.size_of_columns = 0; 
    arg.ang_resolution = 0;
    arg.head = 0;
    arg.tail = 0;
    arg.current_size = 0;
    return *this; //restituisco self-reference
  }

/*La funzione new_scan che accetta un vector<double> contenente i dati forniti dal sensore
e lo memorizza nel buffer (sovrascrivendo il dato più vecchio se il buffer è pieno) – questa
funzione esegue anche il controllo di dimensione: se i dati sono in numero minore del
previsto, completa i dati mancanti a 0, se sono in numero maggiore, li taglia*/
  void LaserScannerDriver::new_scan(const std::vector<double> &v){
    if(!buffer){ //controllo che il buffer esista
      std::cout<<"! INVALID BUFFER !\n";
      return;
    }

    if(isFull()){ //gestione buffer circolare pieno
      tail = head;
      head++;
      if(head > n_of_columns -1){
        tail = n_of_columns-1;
        head = 0;
      }
    }

    delete[] buffer[tail]; //dealloco e libero la colonna sulla quale andrò a scrivere
    //buffer[tail] = nullptr;

    buffer[tail] = new double[size_of_columns]; //alloco la colonna sulla quale andrò a scrivere
    int i = 0;
    for(i=0; i < size_of_columns && i<v.size(); i++) //se il vettore è troppo lungo viene troncato 
      buffer[tail][i] = v[i]; //copio gli elementi del vettore nella colonna

    while(i<size_of_columns){ //se il vettore è troppo corto vengono inseriti zeri
      buffer[tail][i] = 0.0;
      i++;
    }

    if(current_size < n_of_columns) //se il buffer non è pieno incremento la taglia
      current_size++;
    tail++; //aggiorno tail

    if(tail > n_of_columns-1) //eventuale giro di tail
      tail = 0;

    if(tail == head){ //gestione head e tail
      head++;
      if(head > n_of_columns-1)
        head = 0;
    }
  }
  
/*La funzione get_scan che fornisce in output (in maniera opportuna) un vector<double>
contenete la lettura più vecchia del sensore e la rimuove dal buffer*/
  std::vector<double> LaserScannerDriver::get_scan(){
    if(!buffer){ //controllo che il buffer esista
      std::cout<<"! INVALID BUFFER !\nReturned an empty std::vector\n";
      std::vector<double> tmp(0);
      return tmp;
    }

    if(isEmpty()){ //se il buffer è vuoto non posso fare estrazioni
      std::cout<<"! THE BUFFER IS EMPTY ! \nReturned an empty std::vector \n";
      std::vector<double> tmp(0);
      return tmp;
    }

    std::vector<double> v; //dichiaro vettore da ritornare
    for(int i=0; i<size_of_columns; i++){
      v.push_back(buffer[head][i]);
      buffer[head][i] = 0.0; //libero la colonna
    }
    delete[] buffer[head]; //dealloco la colonna
    buffer[head] = nullptr;
    head++; //aggiorno head

    if(head > n_of_columns-1) //eventuale giro di head
      head = 0;

    current_size--; //decremento la taglia
    return v;
  }

/*La funzione clear_buffer che elimina (senza ritornarle) tutte le letture salvate*/
  void LaserScannerDriver::clear_buffer(){
    if(!buffer){ //controllo che il buffer esista
      std::cout<<"! INVALID BUFFER !\n";
      return;
    }

    while(!isEmpty()) 
      get_scan();

    current_size = 0;
    head = tail = 0;
    if(isEmpty())
      std::cout<<"Buffer cleared correctly\n";
  }

/*La funzione get_distance che accetta un angolo (double) e ritorna il valore di distanza
(cioè la lettura) corrispondente a tasa implementele angolo (tenendo conto della risoluzione angolare)
nella scansione più recente acquisita dal sensore – tale scansione non è eliminata dal
buffer, e se l’angolo richiesto non è disponibile la funzione ritorna il valore di angolo più
vicino*/
  const double LaserScannerDriver::get_distance(double angle){
    if(!buffer){ //controllo che il buffer esista
      std::cout<<"! INVALID BUFFER ! \n Returned -1 \n";
      return -1;
    }

    if(isEmpty()){ //controllo se il buffer è vuoto
      std::cout<<"! THE BUFFER IS EMPTY ! \n Returned -1 \n";
      return -1;
    }

    double my_angle = angle;
    if(angle>180){
      std::cout<<"(ANGLE OVER MAX, SETTED at 180) "; //il Lidar legge fino a 180°
      my_angle = 180;
    }

    if(angle<0){
      std::cout<<"(ANGLE OVER MIN, SETTED at 0) "; //il Lidar legge da 0°
      my_angle = 0;
    }

    int index = (int)round(my_angle/ang_resolution);
    if(index > size_of_columns-1)
      index = size_of_columns-1;

    std::vector<double> v = get_last_scan();
    std::cout<<"Input angle: "<< angle <<"\tRounded angle: "<<index*ang_resolution;
    std::cout<<"\tdistance: "<<v[index]<<"\n";
    return v[index];
  }

/*L’overloading dell’operator<< che stampa a schermo l’ultima scansione salvata (ma non la
rimuove dal buffer).*/
  std::ostream& operator<<(std::ostream& os, const LaserScannerDriver& arg){
    std::vector<double> tmp = arg.get_last_scan();
    for(int i=0; i<tmp.size(); i++){
      os << "Angle: " << i*arg.get_ang_resolution()<< "\t Distance: " << tmp[i] << "\n";
    }
    return os;
  }

//utility
//Ritorna il numero di colonne
  const int LaserScannerDriver::get_n_of_columns() const{
    return n_of_columns;
  }

//Ritorna la risoluzione angolare
  const double LaserScannerDriver::get_ang_resolution() const{
    return ang_resolution;
  }

//Ritorna l'indice della scansione più vecchia
  const int LaserScannerDriver::get_head() const{
    return head;
  }

//Ritorna l'indice della scansione più recente
  const int LaserScannerDriver::get_tail() const{
    return tail;
  }

//Ritorna la dimensione delle colonne
  const int LaserScannerDriver::get_size_of_columns() const{
    return size_of_columns;
  }

//Ritorna l'ultima scansione inserita nel buffer senza rimuoverla
  const std::vector<double> LaserScannerDriver::get_last_scan() const{
    std::vector<double> tmp;
    for(int i=0; i<size_of_columns; i++){
      if(tail!=0)
        tmp.push_back(buffer[tail-1][i]);
      else
        tmp.push_back(buffer[n_of_columns-1][i]);
    }
    return tmp;
  }

//Verifica se il buffer è vuoto
  const bool LaserScannerDriver::isEmpty(){
    return (head == tail); //potevo anche usare return current_size == 0;
  }

//Verifica se il buffer è pieno
  const bool LaserScannerDriver::isFull(){
    if(tail +1 == head) //potevo anche usare if(current_size == size_of_columns)
      return true;

    return false;
  }

//Ritorna il numero di scansioni presenti nel buffer attualmente
  const int LaserScannerDriver::get_current_size() const{
    return current_size;
  }

//Stampa i dettagli del buffer passato come argomento
  void print_state(const LaserScannerDriver& lsd ){
    std::cout<<"--------------------------\n"; 
    std::cout<<"-- DETAILS ABOUT BUFFER --\n";
    std::cout<<"Number of columns: "<<lsd.get_n_of_columns()<<"\n";
    std::cout<<"Angular resolution: "<<lsd.get_ang_resolution()<<"\n";
    std::cout<<"Size of columns: "<<lsd.get_size_of_columns()<<"\n";
    std::cout<<"Head: "<<lsd.get_head();
    std::cout<<"\t"<<"Tail: "<<lsd.get_tail()<<"\n";    
    std::cout<<"Current size: "<<lsd.get_current_size()<<"\n";
    std::cout<<"--------------------------\n";          
  }
