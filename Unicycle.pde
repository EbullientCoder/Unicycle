/*
Progetto Uniciclo-GPS

Questo programma si basa sul KalmanUniciclo visto a lezione ,ma sostituisce all'uso dei landmarks 
un reticolo gps creato con un certo passo qstep che può essere modifico dall'utente.
Esistono due modalità:
-Bussola On
-Bussola Off
Se la bussola è attiva si utilizzano per il calcolo del filtro di Kalman le matrici:
Kon,innovazioneOn,RsOn,Hon
Se la bussola è disattiva si utilizzano per il calcolo del filtro di Kalman le matrici:
Koff,innovazioneOff,RsOff,Hoff
Abbiamo creato due funzioni distinte per il calcolo di riga e colonne di cui si avanza o si arretra ,considerando come punto di partenza 0
la riga e la colonna del riquadro centrale.
E' presente una funzione per la colorazione del riquadro che viene utilizzata sia per il riquadro su cui si trova il robot reale sia per 
il riquadro di destinazione.
Il reticolo è disegnato da una funzione gps a  cui si passano come parametri le dimensioni della finestra e il passo qstep.

*/
// Dimensioni finestra
int sizeX = 1000;
int sizeY =  550;

// Coordinate attuali uniciclo
float x = 0;
float y = 0;
float theta = 0;

// Coordinate desiderate uniciclo
float xDes = 0;
float yDes = 0;

// Caratteristiche fisiche uniciclo
float r = 8; // raggio ruote in pixel
float d = 25; // distanza tra le ruote in pixel
float w = 5; // spessore ruota in pixel
float R = 1.2*sqrt(pow(r,2)+pow(d/2+w/2,2)); // raggio robot

float dt = (float) 1/60; // tempo di campionamento

float e_p = 0; // errore posizionamento
float v1 = 0; // velocità lineare robot
float kv1 = 1; // costante legge proporzionale controllo v1
float v2 = 0; // velocità rotazionale robot
float kv2 = 20;  // costante legge proporzionale controllo v2
int nGiri = 0; // conta quanti giri su se stesso ha fatto il robot
float thetaDes; // orientamento desiderato (per raggiungere il target)
float t0,tempo; // tempo inizio e attuale missione
float tempoUltimaMisura; // tempo in cui si è effettuata l'ultima misura
long nStime = 0; // numero progressivo totale delle stime fatte

float omegaR = 0; // velocità angolare ruota destra
float omegaL = 0; // velocità angolare ruota sinistra
float uR, uL, uRe, uLe; // spostamenti ruote destra e sinistra veri e presunti (comandati)

// Variabili relative alla stima e al filtro di Kalman esteso
float sigmaX0 = 10; // deviazione standard dell'errore di stima del valore iniziale di x0
float sigmaY0 = 10; // deviazione standard dell'errore di stima del valore iniziale di y0
float sigmaTheta0 = 10*PI/180; // deviazione standard (in rad) dell'errore di stima del valore iniziale di theta0
float xHat = x + Gaussian(0,sigmaX0); // stima di x inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaX0
float yHat = y + Gaussian(0,sigmaY0); // stima di y inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaY0
float thetaHat = theta + Gaussian(0,sigmaTheta0); // stima di theta inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaTheta0 rad
float xHatMeno,yHatMeno,thetaHatMeno; // stime a priori di x,y,theta
float KR = 0.01; // coefficiente varianza errore odometrico ruota destra
float KL = KR; // coefficiente varianza errore odometrico ruota sinistra
float stDevR,stDevL; // deviazione standard errore odometrico ruota destra e sinistra
boolean bussola=true;

             
// Seguono le matrici utilizzate dal filtro di Kalman esteso (EKF)
float[][] F = {{1, 0, 0},{0, 1, 0},{0, 0, 1}}; // matrice giacobiana F=df/dx (alcuni elementi delle prime due righe vanno aggiustati durante l'esecuzione)
float[][] P = {{pow(sigmaX0,2), 0, 0},{0, pow(sigmaY0,2), 0},{0, 0, pow(sigmaTheta0,2)}}; // matrice di covarianza P inizializzata in base all'incertezza iniziale
float[][] Pmeno = new float[3][3]; // matrice di covarianza a priori P-

float[][] W = {{1, 0},{0,1},{1/d,-1/d}}; //  matrice giacobiana W = df/dw (gli elementi delle prime due righe vanno aggiustati durante l'esecuzione) 
float[][] Q = {{1, 0},{0,1}}; // matrice di covarianza del rumore di misura odometrico w (gli elementi sulla diagonale vanno aggiustati durante l'esecuzione)
float[][] Hon = idMat(3,1); // matrice giacobiana H = dh/dx
float[][] Hoff={{1,0,0},{0,1,0}};

float[][] Kon = new float[3][3]; // guadagno di Kalman
float[][] Koff=new float[3][2];



float tStep = 100000; // tempo (in ms) tra una misura e la successiva (impostabile da tastiera)
float colonna,riga;
float sigmap=10;
int qstep=100;
float sigmab=10*PI/180;
float nb;
float nx;
float ny;

float Nx;
float Ny;

float zx,zy,ztheta;

float[][] Rson ={{pow(sigmap,2)+pow(qstep,2)/12,0,0},{0,pow(sigmap,2)+pow(qstep,2)/12,0},{0,0,pow(sigmab,2)}};  // matrice di covarianza errore misura 
float[][] Rsoff={{pow(sigmap,2)+pow(qstep,2)/12,0},{0,pow(sigmap,2)+pow(qstep,2)/12}};




float[][] innovazioneon = new float[3][1]; // innovazione EKF
float[][] innovazioneoff= new float[2][1];

float[][] correzione = new float[3][1]; // termine correttivo stima



void setup() 
{
  size(1000,550);
  tempoUltimaMisura = 0; // Inizializzo a zero il tempo in cui è stata effettuata l'ultima misura
}

void draw() 
{
  
  background(0);

  pushMatrix();
  translate(sizeX/2,sizeY/2);

  if (keyPressed)
  {
    if (keyCode == UP) // aumento di 1 il tempo tra una misura e la successiva
    {
      tStep += 1;
    }
    if (keyCode == DOWN)  // decremento di 1 il tempo tra una misura e la successiva
    {
      tStep = max(0,tStep-1);
    }
    if (keyCode == RIGHT) // moltiplico per due il tempo tra una lettura e la successiva
    {
      tStep = tStep*2;
    }
    if (keyCode == LEFT) // divido per due il tempo tra una lettura e la successiva
    {
      tStep = tStep/2;
    }
    if (key == '+')
    {
      qstep += 1;
      if(bussola==true){
        Rson[0][0]=pow(sigmap,2)+pow(qstep,2)/12;
        Rson[1][1]=pow(sigmap,2)+pow(qstep,2)/12;
      }else{
        Rsoff[0][0]=pow(sigmap,2)+pow(qstep,2)/12;
        Rsoff[1][1]=pow(sigmap,2)+pow(qstep,2)/12;
        
      }
    }
   if (key == '-')
    {
      qstep -= 1;
      if (qstep < 10)
      {
        qstep = 10;
      }
      
      if(bussola==true){
        Rson[0][0]=pow(sigmap,2)+pow(qstep,2)/12;
        Rson[1][1]=pow(sigmap,2)+pow(qstep,2)/12;
      }else{
        Rsoff[0][0]=pow(sigmap,2)+pow(qstep,2)/12;
        Rsoff[1][1]=pow(sigmap,2)+pow(qstep,2)/12;
        
      }
    }
    if(key=='b'){
      bussola=false;
    }
    if(key=='B'){
     bussola=true; 
     
    }
    
  }
  
  if (mousePressed) // assegno target
  {
    xDes = mouseX - sizeX/2;
    yDes = sizeY/2 - mouseY;
    t0 = millis(); // inizio conteggio del tempo di missione
  }

// Calcolo errore e controllo basandomi sul valore stimato (xHat,yHat,thetaHat) delle variabili dell'uniciclo
  e_p = sqrt(pow(xDes-xHat,2)+pow(yDes-yHat,2));
  if (e_p > 1) // mi muovo solo se l'errore è maggiore di una certa quantità
  {
    tempo = (millis()-t0)/1000;  // tempo missione in secondi

    // assegno velocità secondo legge proporzionale (in termini delle quantità stimate!)
    v1 = -kv1*((xHat-xDes)*cos(thetaHat) + (yHat-yDes)*sin(thetaHat));

    // Calcolo l'angolo verso il target: scelgo il multiplo di 2PI 
    // più vicino all'orientamento corrente ***stimato*** del robot
    thetaDes = atan2(yDes-yHat,xDes-xHat) + nGiri*2*PI;
    if (abs(thetaDes+2*PI-thetaHat) < abs(thetaDes-thetaHat))
    {
      thetaDes = thetaDes+2*PI;
      nGiri += 1;
    }
    else
    {
      if (abs(thetaDes-2*PI-thetaHat) < abs(thetaDes-thetaHat))
      {
        thetaDes = thetaDes-2*PI;
        nGiri += -1;
      }
    }
    
   // assegno velocità angolare secondo legge proporzionale sempre in funzione delle quantità stimate   
    v2 = kv2*(thetaDes-thetaHat);
  }
  else // se penso di essere vicino al target mi fermo
  {
     v1 = 0;
     v2 = 0;
     
  }
  
  // Calcolo i movimenti da impartire alle ruote in base alle v1 e v2 trovate
  omegaR = (v1+v2*d/2)/r;
  omegaL = (v1-v2*d/2)/r;
  uRe = r*omegaR*dt; // spostamento comandato alla ruota destra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  uLe = r*omegaL*dt; // spostamento comandato alla ruota sinistra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  // Perturbo i due movimenti: gli spostamenti reali delle ruote non saranno mai esattamente uguali a quelli comandati 
  stDevR = sqrt(KR*abs(uRe));
  stDevL = sqrt(KL*abs(uLe));  
  uR = uRe + Gaussian(0,stDevR); // Spostamento vero ruota destra
  uL = uLe + Gaussian(0,stDevL); // Spostamento vero ruota sinistra
  
  // Dinamica effettiva dell'uniciclo
  x = x + ((uR+uL)/2)*cos(theta);
  y = y + ((uR+uL)/2)*sin(theta);
  theta = theta + (uR-uL)/d;
  
  // STIMA FILTRO KALMAN ESTESO: PASSO di PREDIZIONE
  xHatMeno = xHat + ((uRe+uLe)/2)*cos(thetaHat);
  yHatMeno = yHat + ((uRe+uLe)/2)*sin(thetaHat);
  thetaHatMeno = thetaHat + (uRe-uLe)/d;
  
  
  //Aggiorno la giacobiana F (solo gli elementi variabili)
  F[0][2] = -(uRe+uLe)*sin(thetaHat)/2;
  F[1][2] = (uRe+uLe)*cos(thetaHat)/2;
  
  // Aggiorno W (solo gli elementi variabili)
  W[0][0] = .5*cos(thetaHat);
  W[0][1] = .5*cos(thetaHat);
  W[1][0] = .5*sin(thetaHat);
  W[1][1] = .5*sin(thetaHat);

  //Aggiorno Q (solo gli elementi variabili)
  Q[0][0] = KR*abs(uRe);
  Q[1][1] = KL*abs(uLe);
  
  
  

  // Calcolo matrice covarianza a priori
  Pmeno = mSum(mProd(mProd(F,P),trasposta(F)),mProd(mProd(W,Q),trasposta(W))); // Pmeno = F*P*F' + W*Q*W'

 // STIMA FILTRO DI KALMAN ESTESO: PASSO di CORREZIONE
  if (millis()-tempoUltimaMisura >= tStep) // attuo la correzione solo se ho le misure (che arrivano ogni tStep ms)
  {
    tempoUltimaMisura = millis(); // memorizzo il tempo in cui ho fatto l'ultima misura
    nStime++; // incremento il contatore delle stime fatte
     
    riga=rect_x(x,qstep);
    colonna=rect_y(y,qstep);
    
    nx=Gaussian(0,pow(sigmap,2));
    ny=Gaussian(0,pow(sigmap,2));
    nb=Gaussian(0,pow(sigmab,2));
    
    
    
    //arrotondamento ad intero delle coordinate con errore gaussiano
     zx=qstep*round((x+nx)/qstep);
     zy=qstep*round((y+ny)/qstep);
     
    if(bussola==true){
    //Bussola accesa   
     
     ztheta=theta+nb;
     
     innovazioneon[2][0]=ztheta-thetaHatMeno;
     
     if((rect_x(xHatMeno,qstep)!=rect_x(x,qstep))||(rect_y(yHatMeno,qstep)!=rect_y(y,qstep))){
     
      innovazioneon[0][0]=zx-xHatMeno;
      innovazioneon[1][0]=zy-yHatMeno;
      
      }
      else
      {
      innovazioneon[0][0]=0;
      innovazioneon[1][0]=0;
      
      }  
    // Calcolo guadagno Kalman e aggiorno covarianza
    Kon = mProd(mProd(Pmeno,trasposta(Hon)),invMat(mSum(mProd(mProd(Hon,Pmeno),trasposta(Hon)),Rson)));
    P = mProd(mSum(idMat(3,1),mProd(idMat(3,-1),mProd(Kon,Hon))),Pmeno);


    }else{
    
    //Bussola spenta
    
         
     
     if((rect_x(xHatMeno,qstep)!=rect_x(x,qstep))||(rect_y(yHatMeno,qstep)!=rect_y(y,qstep))){   
     
      innovazioneoff[0][0]=zx-xHatMeno;
      innovazioneoff[1][0]=zy-yHatMeno;
      
      }
      else
      {
      innovazioneoff[0][0]=0;
      innovazioneoff[1][0]=0;
      
      }  
    
    // Calcolo guadagno Kalman e aggiorno covarianza
      Koff = mProd(mProd(Pmeno,trasposta(Hoff)),invMat(mSum(mProd(mProd(Hoff,Pmeno),trasposta(Hoff)),Rsoff)));
      P = mProd(mSum(idMat(3,1),mProd(idMat(3,-1),mProd(Koff,Hoff))),Pmeno);
  
  }

      // Correggo la stima  
    if(bussola==true)  
      correzione = mProd(Kon,innovazioneon);
    else
      correzione=mProd(Koff,innovazioneoff);
    xHat = xHatMeno + correzione[0][0];
    yHat = yHatMeno + correzione[1][0];
    thetaHat = thetaHatMeno + correzione[2][0];    
  

  }
  else  // se non ho misure non correggo nulla
  {
    xHat = xHatMeno;
    yHat = yHatMeno;
    thetaHat = thetaHatMeno;
  
    P = Pmeno;
  
  }
  
  
// FINE EKF



//evidenzio il quadrato della posizione finale
  rect_color(xDes,yDes,qstep,155,155,255);
// evidenzio il quadrato della posizione attuale
  rect_color(x,y,qstep,155,255,255);

//disegno le linee gps
  gps(sizeX,sizeY,qstep);




// Disegno il robot vero e quello stimato
  robot(x,y,theta,1); // l'argomento 1 fa un robot rosso (robot reale)
  
  
  robot(xHat,yHat,thetaHat,0); // l'argomento 0 un robot giallo (robot nella posa stimata)

  popMatrix();

  textSize(20);
  fill(0,0,255);
  text("v1 (pixel/s) = ",10,20); 
  text(v1,200,20);
  text("v2 (gradi/s) = ",10,50); 
  text(v2*180/PI,200,50);
  
  fill(255,0,0);  
  text("x = ",10,160); 
  text(x,80,160);
  text("y = ",10,190); 
  text(y,80,190);
  text("theta = ",10,220); 
  text(theta*180/PI,100,220);  

  fill(255,255,255);
  text("tempo = ",10,110); 
  text(tempo,120,110);  

  fill(0,0,255);
  text("omegaR (gradi/s) = ",650,20); 
  text(omegaL*180/PI,900,20);
  text("omegaL (gradi/s) = ",650,50); 
  text(omegaL*180/PI,900,50);
  
  fill(255,0,0);
  text("xDes = ",700,100); 
  text(xDes,800,100);
  text("yDes = ",700,130); 
  text(yDes,800,130);

  fill(255,255,0);  
  text("xHat = ",10,280); 
  text(xHat,120,280);
  text("yHat = ",10,310); 
  text(yHat,120,310);
  text("thetaHat = ",10,340); 
  text(thetaHat*180/PI,160,340);  

  fill(255,255,255);
  text("qstep =",10,370);
  text(qstep,120,370);
  text("nStime = ",10,390); 
  text(nStime,120,390);  

  fill(255,255,255);
  text("tStep (ms) = ",10,420); 
  text(tStep,150,420);  
  
  fill(255,255,255);
  text("Bussola = ",700,160); 
  if(bussola==true)
   text("ON",800,160); 
  else
   text("OFF",800,160); 
  
 
 
 
 
  fill(255,255,0);  
  text("P = ",10,460); 
  text(P[0][0],10,490); text(P[0][1],100,490); text(P[0][2],190,490);
  text(P[1][0],10,520); text(P[1][1],100,520); text(P[1][2],190,520); 
  text(P[2][0],10,550); text(P[2][1],100,550); text(P[2][2],190,550);   
  
  
  text("Correzione = ",10,570); 
  
  text(correzione[0][0],10,600);
  text(correzione[1][0],10,640); 
  text(correzione[2][0],10,670);   
    
    
    
    
    
  
  if(bussola==true){
  text("Kon = ",300,460);
  
  text(Kon[0][0],300,490);text(Kon[0][1],380,490);text(Kon[0][2],450,490);
  text(Kon[1][0],300,520);text(Kon[1][1],380,520);text(Kon[1][2],450,520);  
  text(Kon[2][0],300,550);text(Kon[2][1],380,550);text(Kon[2][2],450,550);   
  
  }else{
  
  text("Koff = ",300,460); 
  
  text(Kon[0][0],300,490);text(Kon[0][1],380,490);
  text(Kon[1][0],300,520);text(Kon[1][1],380,520);  
  text(Kon[2][0],300,550);text(Kon[2][1],380,550); 
  
  }

}


void gps(float sizex,float sizey,int passo)  //passo=qstep
{ pushMatrix();
  
  float xline=passo/2;
  float yline=passo/2;
   fill(255,0,0);
   stroke(255,0,0);
   strokeWeight(3);
   line(0,-sizey,0,sizey);  // disegno la colonna di x=0
   line(-sizex,0,sizex,0);  // disegno la riga di y=0
   
   
   
   while(xline<sizex){
     
   
   fill(255,255,255);
   stroke(255,255,255);
   strokeWeight(1);
  
   line(xline,-sizey,xline,sizey); // disegno le colonne a destra di x=0
   line(-xline,-sizey,-xline,sizey);// disegno le colonne a sinistra di x=0
   xline=xline+passo;   //passo alla colonna successiva 
  }
  
   while(yline<sizey){
   
   
   fill(255,255,255);
   stroke(255,255,255);
   strokeWeight(1);
   
   line(-sizex,yline,sizex,yline); // disegno le righe sotto di y=0
   line(-sizex,-yline,sizex,-yline); //disegno le righe sopra di y=0
   yline=yline+passo;   // passo alla riga successiva
  }
  
 popMatrix();
}

 float rect_x(float xdes,int qstep){
  
  boolean searchx=true;
 
  int countx=0;
     int m=0;
     
  //xDes = mouseX - sizeX/2;
  if (xdes>0){
  while(searchx){
     if(xdes>float(qstep/2)+float(qstep)*countx )
       {
         countx=countx+1;
         
       }else{
         m=countx;
         searchx=false;
      }
     }
  }
     if(xdes<0){
  while(searchx){
        
     if(xdes<-float(qstep/2)+float(qstep)*countx )
           {
             countx=countx-1;
           }else{
         m=countx;
         
         searchx=false;
        }
       }
     }
     return riga=m;
    
  }
 
float rect_y(float ydes,int qstep){
     int county=0;
     int n=0;
     boolean searchy=true;     
   
  
  
  if (ydes>0){
  while(searchy){
     
     if(ydes>float(qstep/2)+float(qstep)*county )
       {
         county=county+1;
         
       }else{
         n=county;
         searchy=false;
      }
     }
  }
     if(ydes<0){
  while(searchy){
     if(ydes<-float(qstep/2)+float(qstep)*county )
           {
             county=county-1;
           }else{
         n=county;
         searchy=false;
        }
       }
     }
      return colonna=n;
   }

void rect_color(float xdes,float ydes,int qstep,int colore1,int colore2,int colore3){
  pushMatrix();
      float Zx,Zy;
  fill(colore1,colore2,colore3);
   stroke(colore1,colore2,colore3);
   strokeWeight(3);
   //i primi due parametri trovano l'angolo alto sinistro,gli altri due rispettivamente larghezza e altezza
  if((xdes==0)&&(ydes==0)){
    rect(-qstep/2,-qstep/2,qstep,qstep); 
  }else{
    Zx=float(-qstep/2)+rect_x(xdes,qstep)*float(qstep);
  
    Zy=float(-qstep/2)+rect_y(ydes,qstep)*float(qstep);
  rect(Zx,-Zy-qstep,qstep,qstep);
  }
  popMatrix();
}

void robot(float x, float y, float theta, int colore)
{
// funzione che disegna uniciclo in (x,y) con orientamento theta      
  pushMatrix();
  translate(x,-y);
  rotate(-theta);
  if (colore==1)
  {
    fill(255,0,0);
  }
  else
  {
    fill(255,255,0);
  }
  ellipse(0,0,2*R,2*R); // il robot
  fill(0,0,255);  
  rect(-r,-d/2-w/2,2*r,w);
  rect(-r,d/2-w/2,2*r,w);
  fill(255);
  ellipse(.8*R,0,.2*R,.2*R);
  ellipse(-.8*R,0,.2*R,.2*R);  
  fill(0,255,0);
  triangle(-.1*R,.3*R,-.1*R,-.3*R,.5*R,0);
  popMatrix();
}


/******************************************************
/******************************************************
  DA QUI IN POI CI SONO FUNZIONI DI SERVIZIO: 
  1) CALCOLO ERRORE GAUSSIANO
  2) ALGEBRA MATRICIALE
/******************************************************
/******************************************************/

float Gaussian(float media, float stDev) // Restituisce variabile N(media,stDev^2) approssimata sfruttando il teorema del limite centrale
{
  float somma = 0;
  for (int k=0; k<27; k++) // 27 in modo che sqrt(3/27)=1/3
  {
    somma = somma + random(-(stDev/3),(stDev/3));
  }
  float res=media+somma;
  return res;
}

float[][] mProd(float[][] A,float[][] B) // Calcola prodotto di due matrici A e B (si assume, senza controllarlo, che numero colonne A = numero righe B!)
{
  int nA = A.length;
  int nAB = A[0].length;
  int nB = B[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      for (int k=0; k < nAB; k++) 
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

float[][] mSum(float[][] A,float[][] B) // Calcola la somma di due matrici A e B (si assume, senza controllarlo, che A e B abbiano stesse dimensioni!)
{
  int nA = A.length;
  int nB = A[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
}

float[][] trasposta(float[][] A) // Calcola la trasposta di una matrice A
{
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
  {
    for (int j=0; j < nR; j++) 
    {  
      C[i][j] = A[j][i];
    }
  }
  return C;
}


float[][] minore(float[][] A, int i, int j) // Determina il minore (i,j) di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA-1][nA-1];
  
  for (int iM = 0; iM < i; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM][jM+1];
    } 
  }
  for (int iM = i; iM < nA-1; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM+1][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM+1][jM+1];
    } 
  }
  return C;
}


float det(float[][] A) // Calcola il determinante di A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float determinante = 0;
  
  if (nA == 1)
  {
    determinante = A[0][0];
  }
  else
  {
    for (int j=0; j < nA; j++) 
    {
      determinante = determinante + A[0][j]*pow(-1,j)*det(minore(A,0,j));
    }
  }
  return determinante;
}


float[][] invMat(float[][] A) // Calcola l'inversa di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA][nA];
  float detA = det(A);

/*
  if (abs(detA)<0.001) // Per evitare casi singolari con determinanti vicini a 0
  {
    if (detA>0)
    {
      detA = 0.001;
    }
    else
    {
      detA = -0.001;
    }
  }
*/  
  
  if (nA == 1)
  {
    C[0][0] = 1/detA;
  }
  else
  {
    for (int i=0; i < nA; i++) 
    {
      for (int j=0; j < nA; j++) 
      {
        C[j][i] = pow(-1,i+j)*det(minore(A,i,j))/detA;
      }
    }
  }
  return C;
}

float[][] idMat(int nA, float sigma) // Assegna una matrice identità di ordine nA moltiplicata per una costante sigma
{
  float[][] I = new float[nA][nA]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nA; j++) 
    {  
      I[i][j] = 0;
    }
    I[i][i] = sigma;
  }
  return I;
}
