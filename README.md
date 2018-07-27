 Rastreamento de objetos usando Filtro de Kalman e Visão Computacional

Este trabalho apresenta um algoritmo em tempo real para rastreamento de objetos em visão computacional, usando o Filtro de Kalman como mecanismo de predição para situações de oclusão e ou contaminação da cena por ruído. O principal objetivo deste trabalho é de apresentar de forma didática o desenvolvimento de um algoritmo de rastreamento de objetos baseado em cor. O algoritmo apresentado faz o rastreamento do maior objeto simétrico de uma cor pré-definida presente na cena. É apresentado em detalhes a implementação da etapa de segmentação da imagem, e posteriormente é apresentado uma estratégia para tratar situações com dois objetos da mesma cor. Por fim é demonstrado o uso do Filtro de Kalmam.

INTRODUÇÃO

O rastreamento de objetos é uma das mais importantes áreas da visão computacional, com extensas aplicações tanto para indústria pesada como automobilismo, assim como para a indústria do entretenimento, além de ser uma poderosa ferramenta na área médica (Pinho et al., 2004). Visão computacional consiste em técnicas computacionais no qual possibilita interpretar imagens  (WANGENHEIM et al., 2001). Segundo (Freitas et al.,2010), as principais aplicações do rastreamento de objetos em imagens são para diagnósticos médicos, interfaces Homem-Computador para controle de jogos eletrônicos e na área de segurança, para o monitoramento de ambientes com grandes fluxos de pessoas, tais como aeroportos, plataformas de trens e estacionamentos. O objetivo principal na área de segurança é detectar através dos sistemas de rastreamento de objetos atividades indesejadas, contribuindo para a tomada de decisões dos profissionais de segurança (Relli, 2014).

Um algoritmo de rastreamento de objetos busca a partir de cenas provindas de um sensor óptico, como uma câmera, identificar a trajetória que um ou mais objetos descrevem. No entanto, existem diversos fatores que dificultam a identificação da trajetória descrita por um objeto no mundo real. Seja por variações de iluminação, como o ascender ou apagar de luzes, ruídos de fundo e principalmente oclusões que eventualmente o objeto sofra (Weng et al., 2006). Para contornar as dificuldades do mundo real para o rastreamento de objetos, é feito o uso de diversas estratégias de predição como o Filtro de Partículas e o Filtro de Kalman (Iraei and Faez, 2015).

Os sistemas de rastreamento de objetos usando visão computacional podem ser divido em três estágios, conforme ilustrado na Figura 1. O primeiro estágio é onde ocorre a segmentação da imagem, o segundo estágio é onde faz-se o rastreamento ao longo do tempo do objeto ou alvo (target) e no ultimo estágio, faz-se a classificação dos objetos quanto a suas ações executadas.

Figura 1 - Sistema de rastreamento.


O Filtro de Kalman

O Filtro de Kalman consiste em um conjunto de equações que possibilitam a implementação recursiva de um estimador, gerando predição ótima dos estados futuros de um sistema linear a partir de uma observação presente (Welch and Bishop, 1995). Foi desenvolvido em meados de 1960 por Rudolf Emil Kalman (Kalman et al., 1960), inicialmente para aplicações aeroespaciais. No entanto, logo vislumbraram-se diversas aplicações em outras áreas, como robótica móvel, rastreamento de alvos, identificação de sistemas, controle de processos, análise e processamento de sinais entre outros  (Funk, 2003). Existem hoje variações para sistemas não-lineares, como o Filtro de Kalman Estendido (EKF) e o Filtro de Kalman Unscented (UKF). Neste trabalho será feito o uso do Filtro de Kalman linear (KF) que busca gerar estimativas ótimas dos estados de um sistema descrito por
xk+1=Axk+Buk+wkyk=Cxk+vk(1)(2)

No qual xk∈Rn é o vetor de estados, A∈Rn×n é a matriz de estado, B∈Rn×m é a matriz de entrada, uk∈Rm é o vetor de entrada, wk representa a incerteza associada a modelagem do processo, no qual é assumido como sendo uma distribuição gaussiana, com média nula, yk∈Rp o vetor de saída, C∈Rp×n a matriz de saída e vk a incerteza associada a medição da saída. Da mesma forma, vk é assumido como sendo gaussiano, com média nula e wk e vk não possuem correlação. Para este caso, o filtro de Kalman pode ser implementado por:
x^k+1|kPk+1|kKkx^k+1|k+1Pk+1|k+1=Ax^k|k+Buk=Q+APk|kAT=Pk+1|kCT(R+CPk+1|kCT)−1=x^k+1|k+Kk(yk+1−Cx^k+1|k)=(I−KkC)Pk+1|k(I−KkC)T+KkRKTk(3)(4)(5)(6)(7)

Com P∈Rn×n sendo a matriz de covariância da estimativa, K∈Rm×n o ganho ótimo de Kalman, Q∈Rn×n a matriz de covariância do modelo, R∈Rm×m a matriz de covariância das entradas, I∈Rn×n é a matriz identidade de dimensão comptível e x^k é o vetor de estimativas dos estados no instante k. O Filtro de Kalman funciona em duas etapas, chamadas de predição e correção. Na etapa de predição o filtro gera uma estimativa a priori do vetor de estados, e na etapa de correção, caso disponível, o filtro toma uma medição provinda de um sensor e faz a atualização, gerando uma estimativa a posteriori. Note que nas equações do filtro, a notação k+1|k indica o instante k+1 a priori, ou seja, não possuindo ainda uma medição, enquanto a notação k+1|k+1 indica o instante k+1
dado que já é conhecido uma medição. O ciclo de funcionamento do filtro é ilustrado na Figura 2.

Figura 2 - Ciclo de funcionamento do Filtro de Kalman.

As matrizes Q
e R são parâmetros de sintonia do filtro de Kalman, no qual possibilitam fazer com que ele passe a "confiar" mais na modelagem, conforme ilustrado na Figura 3, ou na medição, conforme ilustrado na Figura 4. As respectivas figuras apresentam as Funções de Densidade de Probabilidade (FDP) da saída do modelo, das medições e da saída do Filtro de Kalman. Para ilustrar o comportamento do filtro, suponha a matriz Q=qI e a matriz R=rI, sendo I a matriz identidade de dimensão compátivel e q,r∈R. Note que para o caso no qual q<r, a FDP do Filtro de Kalman está mais próxima da FDP do modelo. Ou seja, nesse caso, o filtro está tendendo a gerar saídas próximas as do modelo. E no caso que q>r
, o filtro apresenta uma FDP mais próxima da FDP das medidas. Assim a saída do filtro tende a gerar valores próximos aos medidos.

Figura 3 - Confiança maior no modelo (q<r).

Figura 4 - Confinaça maior na medida (q>r).

O objetivo principal deste trabalho é apresentar de forma didática as principais etapas de implementação de um sistema de rastreamento de objetos em tempo real. Fazendo uso de abordagens encontradas na literatura. Para aquisição da imagem, é utilizado uma câmera de baixo custo (webcam) e a plataforma de programação Matlab®, no qual já conta com diversas ferramentas para processamento de imagens. O sistema de rastreamento apresentado visa rastrear o maior objeto na cor vermelha presente na cena. E ainda lidar também com situações de rápidas oclusões, parciais ou totais, através do uso do Filtro de Kalman.

SEGMENTAÇÃO DA IMAGEM

Conforme mencionado anteriormente para todo algoritmo de rastreamento de objetos em visão computacional, existe um estágio de segmentação, de forma a identificar em cada quadro, provindo da câmera, a posição do objeto. Uma das estratégias mais simples para a identificação de objetos numa cena é através de um processo de limiarização. A limiarização é uma das abordagens mais importantes da segmentação de imagens. O princípio da limiarização consiste em separar as regiões da imagem em duas classes, o fundo (background) e o objeto (target) (ARTERO and TOMMASELLI, 2000).

Neste trabalho foi optado por trabalhar com imagens no espaço RGB (Red, Green e Blue). Por ser este trabalho voltado para aplicações em tempo real, o espaço de cores RGB demonstra-se computacionalmente menos custoso, pois em geral, os dispositivos de aquisição de imagens já trabalham neste padrão, não sendo necessário uma etapa de transformação de espaço de cores. Assim as imagens obtidas pelo dispositivo de captura são em geral formadas por três canais de cores, representadas por matrizes. Sendo que as entradas das matrizes são respectivamente a informação relativa ao vermelho, verde e azul para cada pixel, conforme ilustrado na Figura 5.

Figura 5 - Imagem no espaço RGB.

A estratégia de limiarização adotada neste trabalho foi a subtração dos canais de cores verde e azul do canal de cor vermelho, uma vez que busca-se rastrear os objetos na cor vermelha presente na cena. E então considerou-se um valor limiar (threshold), de forma que os pixels resultantes com valores inferiores a este limiar são descartados e os pixels com valores maiores são considerados como parte do objeto a ser rastreado, conforme apresentado na Figura 6.

Figura 6 - Processo de limiarização.

    Obs: O valor de L

    foi obtido empiricamente, através de vários testes. Até chegar no valor ideal para as condições de iluminação no qual a câmera se encontrava no momento da implementação.


Como resultado da limiarização é obtido uma imagem binária, ou seja, cujo os pixels possuem valores de 0 ou 1, resultando em uma imagem do tipo preto e branca, no qual a região branca representa o objeto vermelho presente na cena. Na Figura 7 é apresentado o resultado obtido.

Figura 7 - Resultado da limiarização.

Tratando dois objetos vermelhos na cena

Uma situação possível no qual é desejado que o algoritmo apresente robustez, é no caso de existirem dois objetos na cor vermelha presente na imagem, ou mesmo a presença de pequenos detalhes vermelhos no fundo da imagem. O resultado da limiarização para este caso, possui duas ou mais regiões brancas conforme a Figura 8, no qual apresenta o resultado da limiarização quando é posicionado dois objetos vermelhos diante da câmera.

Figura 8 - Resultado da limiarização para dois objetos vermelhos.

O objetivo deste trabalho é rastrear o maior objeto vermelho presente na cena. Portanto, faz-se necessário a implementação de um mecanismo para buscar a posição do maior objeto vermelho. Visando o mínimo de consumo computacional, de forma a garantir um bom funcionamento em tempo real, foi implementado o algoritmo que faz a acumulação dos pixels da imagem binária, tanto na horizontal, como na vertical. Define-se a imagem binária como sendo uma matriz O∈NN×M definida como O={oij}, com 1≤i≤N e 1≤j≤M, cuja entradas são 0 ou 1. O vetor de acumulação horizontal H∈NN é definido como:
H=[h1 h2 ⋯ hN]T,hi=∑j=1Moij,i=1,2,…,N(8)

E o vetor de acumulação vertical V∈NM como:
V=[v1 v2 ⋯ vM]T,vj=∑i=1Noij,j=1,2,…,M(9)

Dessa forma, o ponto (xmax,ymax) com a maior concentração de pixels vermelhos na imagem é dado por:
xmax=max(V),ymax=max(H)(10)

Na Figura 9 é ilustrado a acumulação dos \textit{pixels} da imagem binária e os respectivos pontos de máximo, que coincidem com o ponto na imagem que contém a maior concentração de pixels vermelhos.

Figura 9 - Acumulação dos pixels na vertical e horizontal.

Após a identificação da região onde possui o maior objeto vermelho na cena, define-se uma região de interesse, formada considerando-se o intervalo de 10%, para cima, para baixo e para os lados, em torno do ponto (xmax,ymax). Em seguida é determinado a coordenada (xc,yc) do objeto aplicando-se o cálculo do centro geométrico na região de interesse, através das equações:
xc=∑Ni=1oiji∑Ni=1∑Mj=1oijyc=∑Mi=1oijj∑Ni=1∑Mj=1oij(11)(12)

TRATAMENTO DE OCLUSÕES

Note que para o caso no qual é possível visualizar o objeto na cena, o procedimento apresentado na seção anterior é suficiente para fazer o rastreamento. Porém, caso este objeto sofra uma oclusão, o procedimento descrito falha em buscar as coordenadas do objeto. Para contornar este problema, foi tomado como ferramenta o Filtro de Kalman e então, nas situações de oclusão, não mais é feito o processamento da imagem, mas é gerado estimativas da posição do objeto baseando-se no ultimo instante no qual foi possível visualizar o objeto.

Para implementar o Filtro de Kalman deve-se considerar um modelo para a dinâmica do movimento do objeto, neste trabalho optou-se por utilizar o modelo linear dado por:
⎡⎣⎢⎢⎢⎢x^1(k+1)x^2(k+1)x^3(k+1)x^4(k+1)⎤⎦⎥⎥⎥⎥=⎡⎣⎢⎢⎢10000100Δt0100Δt01⎤⎦⎥⎥⎥⎡⎣⎢⎢⎢⎢x^1(k)x^2(k)x^3(k)x^4(k)⎤⎦⎥⎥⎥⎥+⎡⎣⎢⎢⎢⎢12Δt20Δt0012Δt20Δt⎤⎦⎥⎥⎥⎥[u1(k)u2(k)],[z1(k)z2(k)]=[10010000]⎡⎣⎢⎢⎢⎢x^1(k)x^2(k)x^3(k)x^4(k)⎤⎦⎥⎥⎥⎥(13)(14)

Sendo que x^1=x^c é a estimativa da coordenada horizontal, x^2=y^c é a estimativa da coordenada vertical, x^3=v^x é a estimativa da velocidade na horizontal, x^4=v^y é a estimativa da velocidade na vertical, u1=ax é a aceleração horizontal e u2=ay é a aceleração vertical. Note que o modelo descreve um movimento retilíneo uniformemente variado (MURV) e o Δt presente, indica o tempo de amostragem, que para este caso, é o tempo no qual o Matlab® leva para processar cada quadro da cena.

    Obs: Os valores de ax

e ay

    são obtidos através dos frames anteriores, fazendo aproximação da derivada segunda da posição. 


Com o modelo definido, pode-se aplicar as equações do Filtro de Kalman apresentadas na introdução e então gerar estimativas para a posição do objeto vermelho. Porém, como é desejado o tratamento de oclusões, foi optado por utilizar não apenas um, mas dois Filtros de Kalman, sendo que o primeiro é sintonizado para ter "confiança" na medição. E o segundo "confiança" no modelo. Assim obtêm-se um algoritmo com maior robustez. A Figura 10 apresenta o diagrama conceitual da estrutura utilizada.

Figura 10 - Diagrama do sistema de filtragem.

Note que a saída passa a ser Y1
e Y2
, que são selecionadas conforme a detecção ou não de oclusões. Nos instantes em que não existe oclusão, a saída é  aquela provinda do Filtro de Kalman que "confia" mais na medição. E quando verifica-se uma oclusão, é selecionado a saída do Filtro de Kalman que "confia" mais no modelo. Esta estratégia foi necessária pois, o filtro sintonizado para confiar no modelo apresenta bons resultados nas situações de oclusão, porém uma baixa eficiência na situações sem oclusão e vice-versa.

RESULTADO

O resultado obtido pelo algoritmo de rastreamento desenvolvido é apresentado nesta seção. Para situações sem oclusão, o resultado é conforme apresentado na Figura 11.

Figura 11 - Resultado do rastreamento.

Para as situações com oclusão, o resultado é apresentado na Figura 12.

Figura 12 - Resultado com oclusões.

Nos instantes em que não é possível visualizar o objeto, é tomado os valores obtidos pelo Filtro de Kalman sintonizado para "confiar" no modelo, então baseando-se no ultimo instante que foi possível visualizar o objeto, é gerado estimativas da trajetória do objeto conforme apresentado na Figura 13.

Figura 13 - Resultado da trajetória estimada.

Note que como o modelo utilizado é linear, a estimativa obtida é de uma trajetória retilínea. No entanto, para situações sem oclusão, por ser utilizado um filtro sintonizado para "confiar" na medição, obtêm-se um bom desempenho para movimentos não-lineares, conforme apresentado na Figura 14.

Figura 14 - Resultado para trajetórias não-lineares.
VÍDEO DEMONSTRAÇÃO
 
CÓDIGO FONTE

https://www.dropbox.com/s/kvuvzkgk6yhf2fq/kalman_live.m?dl=0

CONCLUSÃO

Com o desenvolvimento deste trabalho foi possível verificar na prática o desempenho do Filtro de Kalman para estimar a trajetória de objetos com situações no qual existe falta de informação. Também foi apresentado os principais detalhes de implementação do sistema de visão computacional, voltando-se para a área de rastreamento de objetos. Mostrou-se que é possível obter um desempenho satisfatório para rastreamento de objetos em cenas obtidas por uma câmera de baixo custo, mesmo com a presença de mais de um objeto da mesma cor. O algoritmo apresentou boa eficiência para situações de rápidas oclusões observou-se que este projeto ilustra de forma simples o potencial do Filtro de Kalman e sua relativa simplicidade de implementação.


REFERÊNCIAS

ARTERO, A. and TOMMASELLI, A. (2000). Limiarização automática de imagens digitais, Boletim de Ciências Geodésicas 6(1): 38–48.

Freitas, G. M. et al. (2010). Rastreamento de objetos em vídeos e separação em classes.

Funk, N. (2003). A study of the kalman filter applied to visual tracking, University of Alberta, Project for CMPUT 652(6).

Iraei, I. and Faez, K. (2015). Object tracking with occlusion handling using mean shift, kalman filter and edge histogram, Pattern Recognition and Image Analysis (IPRIA), 2015 2nd International Conference on, IEEE, pp. 1–6.

Kalman, R. E. et al. (1960). A new approach to linear filtering and prediction problems, Journal of basic Engineering 82(1): 35–45.

Pinho, R. R., Tavares, J. M. R. S. and Correia, M. F. P. V. (2004). Introdução à análise de movimento usando visão computacional.

Relli, C. (2014). Caracterização de algoritmos de rastreamento de objetos em video considerando situações de oclusão, RETEC-Revista de Tecnologias 6(1).

Van den Bergh, M. and Van Gool, L. (2011). Combining rgb and tof cameras for real-time 3d hand gesture interaction, Applications of Computer Vision (WACV), 2011 IEEE Workshop on, IEEE,p. 66–72.

WANGENHEIM, A. v. et al. (2001). Seminario introdução a visão computacional, Visão Computacional Aldon von Wangenheim’s HomePage.

Welch, G. and Bishop, G. (1995). An introduction to the kalman filter.

Weng, S.-K., Kuo, C.-M. and Tu, S.-K. (2006). Video object tracking using adaptive kalman filter,Journal of Visual Communication and Image Representation 17(6): 1190–1208.
