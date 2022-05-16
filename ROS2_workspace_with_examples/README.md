# Example code

*English follows Dutch*

Deze workspace bevat de codevoorbeelden uit de reader en nog meer. De codevoorbeelden zijn zo veel mogelijk voorzien van commentaar voor snelle en directe uitleg. De voorbeelden zijn getest met ROS 2 Foxy Fitzroy.

## Inhoud
**- hello_noheaders**  
      Een package met een simpele node in 1 bestand (HelloNode code van de reader).  
**- hello**  
     Een package met dezelfde functionaliteit als hello_noheaders, echter met header en losse main files.  
**- pub_without_spinning**  
     Een voorbeeld van hoe men kan publisheren zonder een spinnende node te maken. De package bevat een subriber die wel spint en luistert naar een topic waar een niet spinnend main-functie op publiceerd.  
**- pubsub**  
    Een voorbeeld van een publisher en subscriber met een std::msg. De code is netjes gesplist in verschillende bestanden.  
**- pubsub_custom_msg_in_pkg**  
    Zelfde als pubsub echter nu met een custom message in de package.  
**- pubsub_msg_from_custom_interfaces**  
    Zelfde als pubsub echter nu met een custom message uit de package custom_interfaces  
**- rviz_example**  
    Een voorbeeld van node die via tf2 en joint states communiceert. Een urdf file die een robot beschrijft en Rviz die het visualiseert. (Sommige mensen lijken problemen te ondervinden met Rviz en cillinders. Oorzaak nog niet gevonden. Voor nu: gebruik dan rectangles)  
**- srvcli_custom_srv_in_pkg**  
    Een service server en een client. Maakt gebruik van de custom service is gedefineerd binnen deze package (zie codevoorbeeld `srvcli_srv_from_custom_interface` voor een custom service die buiten de package is gedefineerd). De call is asynchroon. De client roept bij het opstarten (via de constructor) 1 keer de service aan.  
**- srvcli_libellebil**  
    Uitwerking van een practicumopdracht uit de reader.  
**- srvcli_srv_from_custom_interface**  
    Een service server en een client. Maakt gebruik van een custom service uit de package custom_interfaces. De call is asynchroon. De client roept bij het opstarten (via de constructor) 1 keer de service aan.  
**- actsrv_action_from_custom_interfaces**  
   Een action server en een client. Maakt gebruik van een custom action uit de package custom_interfaces. De call is asynchroon. De action server geef feedback over het uitvoeren van de action.  
**- tf2_example**  
   Een voorbeeld van een tf2 broadcaster en listener. De broadcaster maakt twee frames en de listener vraagt de locatie van 1 frame op in coordinaten van de world frame.  
**- weird_pubsub**  
   Uitwerking van een practicumopdracht uit de reader.  

## Bouwen

Om deze codevoorbeelden werkend te zien bouw dan vanuit deze folder de workspace. Gebruik hiervoor de volgende commando's in de terminal. Hierbij is de aanname dat ROS 2 al is geinstalleerd.

Voeg de ROS 2 enviroment toe aan de terminal:  
`source /location/of/ROSinstallation/ros/foxy/setup.bash`  
Bij standaard installatie wordt het commando:  
`source /opt/ros/foxy/setup.bash`

Ga naar de workspace:  
`cd location/of/workspace/ROS2_workspace_with_examples`

Bouw alle packages:  
`colcon build`

Meer uitleg over het bouwen van packages/nodes kan men vinden in de reader in appendix A.

# English

*todo*
