Sistema de Gestão e Controlo de Movimento CNC
Visão Geral
Este repositório contém os componentes de software utilizados no controlo e operação de uma máquina CNC, abrangendo desde a interface de utilizador até ao firmware executado no microcontrolador. O projeto encontra-se organizado em módulos independentes, responsáveis pela comunicação, processamento de trajetórias, planeamento de movimento e controlo dos atuadores. 
O firmware tem como principal função interpretar instruções G-Code, calcular trajetórias de movimento e gerar sinais de controlo para os motores de forma sincronizada. Para esse efeito, são implementados algoritmos de interpolação geométrica, planeamento cinemático e temporização de pulsos com resolução compatível com os requisitos de precisão da máquina. 
Estrutura do Projeto
O código está organizado em diretórios com responsabilidades bem definidas: 
•	/backend – Camada de lógica de negócio e comunicação entre utilizadores, base de dados e dispositivos. 
•	/src – Implementação dos algoritmos de controlo e processamento CNC em linguagem C. 
•	/firmware – Binários compilados para gravação no microcontrolador. 
Arquitetura de Processamento
O fluxo de execução segue uma sequência de processamento composta por quatro módulos principais: 
1.	Parser de G-Code – Responsável pela interpretação e validação dos comandos recebidos. 
2.	Controlo de Movimento – Executa os cálculos geométricos necessários para movimentos lineares e circulares. 
3.	Planeador de Trajetória – Gere acelerações, desacelerações e otimização da sequência de movimentos. 
4.	Controlador de Motores – Gera os sinais STEP e DIR através de temporizadores e interrupções do microcontrolador. 
Como Gravar o Firmware no Microcontrolador
5.	Descarregar o Projeto: Faz o download de todo o código do repositório em formato comprimido clicando no botão para baixar o ficheiro .zip.
6.	Descompactar os Ficheiros: Extrai o conteúdo do ficheiro .zip descarregado para uma pasta à tua escolha no teu computador.
7.	Abrir no Arduino IDE: Abre a aplicação do Arduino IDE (ou o teu editor C++), navega até à pasta descompactada e abre o ficheiro principal do projeto.
8.	Enviar para a Placa: Conecta a tua placa Arduino ao computador através do cabo USB, seleciona a porta correta no programa e clica no botão "Carregar" (Upload) para enviar o código diretamente para o Arduino.
