#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme d'asservissement en position des moteurs du robot T-Quad,
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.2 - 10/10/2018
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino import *

# Imports Généraux
import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

# Moteurs
Nmoy = 1

thetaArriereDroit = 0.
codeurArriereDroitDeltaPos = 0
codeurArriereDroitDeltaPosPrec = 0

thetaArriereGauche = 0.
codeurArriereGaucheDeltaPos = 0
codeurArriereGaucheDeltaPosPrec = 0

thetaAvantDroit = 0.
codeurAvantDroitDeltaPos = 0
codeurAvantDroitDeltaPosPrec = 0

thetaAvantGauche = 0.
codeurAvantGaucheDeltaPos = 0
codeurAvantGaucheDeltaPosPrec = 0

# Les moteurs sont asservis en position grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
thetaref = 0.
thetarefArriereDroit = 0. # consigne de position
thetarefArriereGauche = 0. # consigne de position
thetarefAvantDroit = 0. # consigne de position
thetarefAvantGauche = 0. # consigne de position

# Tension effectivement appliquée
commandeArriereDroit = 0.
commandeArriereGauche = 0.
commandeAvantDroit = 0.
commandeAvantGauche = 0.

# Saturations
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

I_x = [0., 0., 0., 0.]
D_x = [0., 0., 0., 0.]
erreurprec = [0., 0., 0., 0.] # écart d'asservissement au calcul précédent

Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID


# Variables utilisées pour les données reçues
typeSignal = 0
offset = 0.
amplitude = 0.
frequence = 0.
Kp = 2 # gain proportionnel du PID
Ki = 0 # gain intégral du PID
Kd = 0. # gain dérivé du PID
moteur1 = 0
moteur2 = 0
moteur3 = 0
moteur4 = 0

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
i = 0
tprec = time.time()
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 100

# Mesure de la tension de la batterie
# On la contraint à être supérieure à 7V, pour éviter une division par
# zéro en cas de problème quelconque
lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")


#--- setup --- 
def setup():
    CommandeMoteurs(0, 0, 0, 0)
    
    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --

def CalculVitesse():
    global thetaArriereDroit, thetaArriereGauche, thetaAvantDroit, thetaAvantGauche, timeLastReceived, timeout, timedOut, \
        tdebut, codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, codeurAvantDroitDeltaPos, codeurAvantGaucheDeltaPos, \
        commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche, \
        thetarefArriereDroit, thetarefArriereGauche, thetarefAvantDroit, thetarefAvantGauche, \
        codeurArriereDroitDeltaPosPrec, codeurArriereGaucheDeltaPosPrec, codeurAvantDroitDeltaPosPrec, codeurAvantGaucheDeltaPosPrec, tprec, \
        idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim, \
        typeSignal, offset, amplitude, frequence, moteur1, moteur2, moteur3, moteur4, \
        Kp, Ki, Kd, Tf
    
    tdebut = time.time()
        
    # Mesure de la position des moteurs grâce aux codeurs incrémentaux
    try:
        codeursDeltaPos = mega.read_codeursDeltaPos()
        codeurArriereDroitDeltaPos = codeursDeltaPos[0]
        codeurArriereGaucheDeltaPos = codeursDeltaPos[1]
        codeurAvantDroitDeltaPos = codeursDeltaPos[2]
        codeurAvantGaucheDeltaPos = codeursDeltaPos[3]
        
        # Suppression de mesures aberrantes
        if (abs(codeurArriereDroitDeltaPos - codeurArriereDroitDeltaPosPrec) > 10) or (abs(codeurArriereGaucheDeltaPos - codeurArriereGaucheDeltaPosPrec) > 10) or (abs(codeurAvantDroitDeltaPos - codeurAvantDroitDeltaPosPrec) > 10) or (abs(codeurAvantGaucheDeltaPos - codeurAvantGaucheDeltaPosPrec) > 10):
            codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
            codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
            codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
            codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec

        codeurArriereDroitDeltaPosPrec = codeurArriereDroitDeltaPos
        codeurArriereGaucheDeltaPosPrec = codeurArriereGaucheDeltaPos
        codeurAvantDroitDeltaPosPrec = codeurAvantDroitDeltaPos
        codeurAvantGaucheDeltaPosPrec = codeurAvantGaucheDeltaPos
    except:
        #print "Error getting data"
        codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
        codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
        codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
        codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec
    
    omegaArriereDroit = -2 * ((2 * 3.141592 * codeurArriereDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaArriereGauche = 2 * ((2 * 3.141592 * codeurArriereGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaAvantDroit = -2 * ((2 * 3.141592 * codeurAvantDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaAvantGauche = 2 * ((2 * 3.141592 * codeurAvantGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    
    thetaArriereDroit = thetaArriereDroit + omegaArriereDroit * dt
    thetaArriereGauche = thetaArriereGauche + omegaArriereGauche * dt
    thetaAvantDroit = thetaAvantDroit + omegaAvantDroit * dt
    thetaAvantGauche = thetaAvantGauche + omegaAvantGauche * dt
        
    tcourant = time.time() - T0
    # Calcul de la consigne en fonction des données reçues sur la liaison série
    if typeSignal == 0: # signal carré
        if frequence > 0:
            if (tcourant - (float(int(tcourant*frequence)))/frequence < 1/(2*frequence)):
                thetaref = offset + amplitude
            else:
                thetaref = offset
        else:
            thetaref = offset + amplitude
    else: # sinus
        if frequence > 0:
            thetaref = offset + amplitude * sin(2*3.141592*frequence*tcourant)
        else:
            thetaref = offset + amplitude

    # Application de la consigne sur chaque moteur
    thetarefArriereDroit = moteur1 * thetaref
    thetarefArriereGauche = moteur2 * thetaref
    thetarefAvantDroit = moteur3 * thetaref
    thetarefAvantGauche = moteur4 * thetaref
            
    dt2 = time.time() - tprec
    tprec = time.time()
    
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        
    if timedOut:
        commandeArriereDroit = 0.
        commandeArriereGauche = 0.
        commandeAvantDroit = 0.
        commandeAvantGauche = 0.
    else:
        commandeArriereDroit = -PID(0, thetarefArriereDroit, thetaArriereDroit, Kp, Ki, Kd, Tf, umax, umin, dt2);
        commandeArriereGauche = PID(1, thetarefArriereGauche, thetaArriereGauche, Kp, Ki, Kd, Tf, umax, umin, dt2);
        commandeAvantDroit = -PID(2, thetarefAvantDroit, thetaAvantDroit, Kp, Ki, Kd, Tf, umax, umin, dt2);
        commandeAvantGauche = PID(3, thetarefAvantGauche, thetaAvantGauche, Kp, Ki, Kd, Tf, umax, umin, dt2);
    
    CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche)
    
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1
        
    #print time.time() - tdebut

    
def PID(iMoteur, ref, mes, Kp, Ki, Kd, Tf, umax, umin, dt2):
    global I_x, D_x, erreurprec
    
    # Calcul du PID
    # Paramètres intermédiaires
    br = 1. / (Kp + 0.0001)
    ad = Tf / (Tf + dt2);
    bd = Kd / (Tf + dt2);

    # Calcul de la commande avant saturation
    erreur = ref - mes
    
    # Terme proportionnel
    P_x = Kp * erreur

    # Terme dérivé
    D_x[iMoteur] = ad * D_x[iMoteur] + bd * (erreur - erreurprec[iMoteur])

    # Calcul de la commande avant saturation
    if (Ki == 0):
        I_x[iMoteur] = 0.
    commande_avant_sat = P_x + I_x[iMoteur] + D_x[iMoteur]

    # Application de la saturation sur la commande
    if (commande_avant_sat > umax):
        commande = umax
    elif (commande_avant_sat < umin):
        commande = umin
    else:
        commande = commande_avant_sat
    
    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x[iMoteur] = I_x[iMoteur] + Ki * dt2 * (erreur + br * (commande - commande_avant_sat))
    
    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    erreurprec[iMoteur] = erreur
    
    return commande


def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(tension_int_AvantDroit, tension_int_AvantGauche)
        mega.moteursCRC(tension_int_ArriereDroit + tension_int_ArriereGauche, tension_int_AvantDroit + tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"

    
def emitData():
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    #delay(5000)
    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global thetarefArriereDroit, thetarefArriereGauche, thetarefAvantDroit, thetarefAvantGauche, \
            typeSignal, offset, amplitude, frequence, Kp, Ki, Kd, moteur1, moteur2, moteur3, moteur4, timeLastReceived, timedOut
            
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('typeSignal') != None:
            typeSignal = int(jsonMessage.get('typeSignal'))
        if jsonMessage.get('offset') != None:
            offset = float(jsonMessage.get('offset'))
        if jsonMessage.get('amplitude') != None:
            amplitude = float(jsonMessage.get('amplitude'))
        if jsonMessage.get('frequence') != None:
            frequence = float(jsonMessage.get('frequence'))
        if jsonMessage.get('Kp') != None:
            Kp = float(jsonMessage.get('Kp'))
        if jsonMessage.get('Ki') != None:
            Ki = float(jsonMessage.get('Ki'))
        if jsonMessage.get('Kd') != None:
            Kd = float(jsonMessage.get('Kd'))
        if jsonMessage.get('moteur1') != None:
            moteur1 = float(jsonMessage.get('moteur1'))
        if jsonMessage.get('moteur2') != None:
            moteur2 = float(jsonMessage.get('moteur2'))
        if jsonMessage.get('moteur3') != None:
            moteur3 = float(jsonMessage.get('moteur3'))
        if jsonMessage.get('moteur4') != None:
            moteur4 = float(jsonMessage.get('moteur4'))
        
        if not socketOK:
            pass
  

    def on_close(self):
        print 'connection closed...'
        socketOK = False

    def sendToSocket(self):
        global socketOK, commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche, \
            thetaArriereDroit, thetaArriereGauche, thetaAvantDroit, thetaAvantGauche, distance, distanceFiltre, \
            thetarefArriereDroit, thetarefArriereGauche, thetarefAvantDroit, thetarefAvantGauche
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'thetaRefArriereDroit':("%.2f" % thetarefArriereDroit), \
                                'thetaRefArriereGauche':("%.2f" % thetarefArriereGauche), \
                                'thetaRefAvantDroit':("%.2f" % thetarefAvantDroit), \
                                'thetaRefAvantGauche':("%.2f" % thetarefAvantGauche), \
                                'thetaArriereDroit':("%.2f" % thetaArriereDroit), \
                                'thetaArriereGauche':("%.2f" % thetaArriereGauche), \
                                'thetaAvantDroit':("%.2f" % thetaAvantDroit), \
                                'thetaAvantGauche':("%.2f" % thetaAvantGauche), \
                                'commandeArriereDroit':("%.2f" % -commandeArriereDroit), \
                                'commandeArriereGauche':("%.2f" % commandeArriereGauche), \
                                'commandeAvantDroit':("%.2f" % -commandeAvantDroit), \
                                'commandeAvantGauche':("%.2f" % commandeAvantGauche), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % thetarefArriereDroit) \
                                + "," + ("%.2f" % thetarefArriereGauche) \
                                + "," + ("%.2f" % thetarefAvantDroit) \
                                + "," + ("%.2f" % thetarefAvantGauche) \
                                + "," + ("%.2f" % thetaArriereDroit) \
                                + "," + ("%.2f" % thetaArriereGauche) \
                                + "," + ("%.2f" % thetaAvantDroit) \
                                + "," + ("%.2f" % thetaAvantGauche) \
                                + "," + ("%.2f" % commandeArriereDroit) \
                                + "," + ("%.2f" % commandeArriereGauche) \
                                + "," + ("%.2f" % commandeAvantDroit) \
                                + "," + ("%.2f" % commandeAvantGauche) \
                                })
                                
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    print 'Sortie du programme'
    CommandeMoteurs(0, 0, 0, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


