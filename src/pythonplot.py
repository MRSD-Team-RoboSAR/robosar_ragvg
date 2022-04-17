import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import cv2
img = cv2.imread('/home/naren/catkin_ws/src/robosar_ragvg/maps/scott_final.png')
# x = [[647, 375], [626, 368], [523, 367], [628, 361], [188, 355], [714, 351], [348, 346], [726, 335], [741, 331], [657, 330], 752, 260, 348, 256, 791, 253, 322, 250, 367, 244, 799, 239, 808, 211, 793, 206, 790, 195, 771, 174, 773, 149, 811, 146, 730, 143, 754, 135, 726, 131, 755, 95, 707, 93, 721, 88, 764, 70, 794, 42, 764, 39, 775, 30, 757, 26, 812, 25, 731, 24, 759, 17, 722, 2, 746, 2, 764, 2, 163, 442, 529, 355, 610, 334, 229, 314, 844, 302, 415, 298, 849, 279, 697, 241, 247, 211, 274, 206, 375, 235, 814, 181, 714, 179, 817, 123, 701, 100, 754, 1, 773, 1
# a = np.zeros((56,2))
# a = np.array([[647,375],[626,368],[523,367],[628,361],[188,355],[714,351],[348,346],[726,335],[741,331],[657,330],[752,260],[348,256],[791,253],[322,250],[367,244],[799,239],[808,211],[793,206],[790,195],[771,174],[773,149],[811,146],[730,143],[754,135],[726,131],[755,95],[707,93],[721,88],[764,70],[794,42],[764,39],[775,30],[757,26],[812,25],[731,24],[759,17],[722,2],[746,2],[764,2],[163,442],[529,355],[610,334],[229,314],[844,302],[415,298],[849,279],[697,241],[247,211],[274,206],[375,235],[814,181],[714,179],[817,123],[701,100],[754,1],[773,1]])
# a = np.array([[647,375],[523,367],[188,355],[714,351],[348,346],[657,330],[752,260],[348,256],[791,253],[808,211],[771,174],[811,146],[730,143],[755,95],[707,93],[794,42],[763,38],[731,24],[764,2],[163,442],[610,334],[229,314],[844,302],[415,298],[697,241],[247,211],[714,179]])
# a = np.array([[647,375],[523,367],[188,355],[714,351],[348,346],[657,330],[752,260],[348,256],[791,253],[808,211],[771,174],[811,146],[730,143],[755,95],[707,93],[794,42],[763,38],[731,24],[764,2]])
# a = np.array([[647,375],[523,367],[188,355],[714,351],[348,346],[752,260],[348,256],[808,211],[773,149],[755,95],[794,42],[731,24]])
# a = np.array([[66,286],[442,261],[340,238],[541,238],[227,225],[495,170],[227,167],[505,114],[471,56],[496,2]]) # window = 50
# a = np.array([[66,286],[442,261],[55,257],[96,253],[340,238],[541,238],[412,237],[478,234],[514,227],[227,225],[495,170],[227,167],[521,164],[473,139],[505,114],[479,92],[490,66],[500,36],[496,2]]) #25
# a = np.array([[66,286],[442,261],[55,257],[96,253],[340,238],[541,238],[412,237],[478,234],[514,227],[227,225],[495,170],[227,167],[521,164],[473,139],[505,114],[479,92],[490,66],[500,36]]) #25 last point removed
# a = np.array([[66,286],[442,261],[340,238],[541,238],[227,225],[495,170],[227,167],[505,114],[471,56]]) #50 with last point removed
# a = np.array([[96,253],[95,252],[96,252],[442,243],[441,242],[442,242],[447,236],[525,227],[518,147],[479,92],[500,36]]) #5 points
# a = np.array([[442,261],[55,257],[96,253],[95,252],[96,252],[74,246],[75,246],[74,245],[442,243],[441,242],[442,242],[340,238],[447,236],[478,234],[514,227],[525,227],[495,170],[494,169],[495,169],[520,164],[521,164],[520,163],[473,139],[505,114],[479,92],[490,66],[499,48],[500,48],[499,47],[491,22]])
# a = np.array([[55,257],[96,253],[95,252],[96,252],[442,243],[441,242],[442,242],[340,238],[447,236],[514,227],[227,225],[520,164],[521,164],[520,163],[487,125],[487,124],[488,124],[479,92],[490,66],[491,22]])
a = np.array([[55,257],[55,257],[96,253],[95,252],[96,252],[442,243],[442,243],[441,242],[442,242],[340,238],[340,238],[447,236],[514,227],[514,227],[227,225],[227,225],[520,164],[520,164],[521,164],[520,163],[487,125],[487,124],[488,124],[479,92],[490,66],[491,22]])
# a = np.array([[55,257],[96,253],[442,243],[340,238],[514,227],[227,225],[520,164],[487,125],[479,92],[490,66],[491,22],[66,286],[442,261],[74,246],[44,241],[541,238],[412,237],[478,234],[525,227],[495,170],[227,167],[518,147],[473,139],[505,114],[506,99],[490,85],[471,56],[499,48],[500,36]])
# a = np.array([[442,261],[55,257],[96,253],[340,238],[447,236],[478,234],[514,227],[495,170],[520,164],[473,139],[505,114],[479,92],[490,66],[491,22],[66,286],[541,238],[412,237],[227,225],[227,167],[495,1]])
a = np.array([[66,286],[442,261],[55,257],[96,253],[74,246],[442,243],[340,238],[541,238],[478,234],[514,227],[525,227],[227,225],[495,170],[520,164],[518,147],[473,139],[487,125],[505,114],[506,99],[479,92],[490,85],[490,66],[499,48],[500,36],[491,22],[44,241],[412,237],[227,167],[471,56]])
a = np.array([[66,286],[442,261],[55,257],[96,253],[340,238],[541,238],[447,236],[478,234],[514,227],[227,225],[495,170],[520,164],[473,139],[505,114],[479,92],[490,66],[500,36],[412,237],[227,167]])
a = np.array([[0,286],[0,261],[0,239],[0,216],[0,170],[0,147],[0,125],[0,99],[0,66],[0,36],[66,286],[442,261],[97,252],[96,251],[97,251],[442,243],[441,242],[442,242],[37,239],[36,238],[37,238],[78,239],[340,238],[541,238],[412,237],[413,237],[412,236],[626,237],[627,237],[627,236],[446,236],[446,235],[447,235],[483,228],[514,227],[525,227],[227,225],[495,170],[494,169],[495,169],[227,167],[520,164],[521,164],[520,163],[518,147],[473,140],[472,139],[473,139],[487,125],[487,124],[488,124],[505,114],[506,99],[479,92],[490,85],[490,66],[471,56],[499,48],[500,48],[499,47],[500,36],[491,22],[496,2],[495,1],[496,1],[497,1]])
a = np.array([[0,286],[0,252],[0,216],[0,170],[0,139],[0,99],[0,66],[0,22],[66,286],[442,261],[97,252],[37,239],[340,238],[541,238],[626,237],[483,228],[227,225],[495,170],[227,167],[472,139],[505,114],[490,66],[491,22]])
a = np.array([[441,257],[442,255],[442,254],[442,253],[442,252],[442,251],[442,250],[442,249],[442,248],[442,247],[442,246],[442,245],[442,244],[442,243],[541,244],[540,243],[539,243],[538,243],[537,243],[536,243],[535,243],[534,243],[533,242],[532,242],[531,241],[530,240],[529,239],[528,238],[527,237],[526,236],[525,235],[524,234],[523,233],[522,232],[521,231],[520,230],[519,229],[542,243],[543,243],[544,243],[545,243],[546,243],[547,243],[548,243],[549,243],[550,243],[551,243],[552,243],[553,243],[554,243],[555,243],[556,243],[557,243],[558,243],[559,243],[560,243],[561,243],[562,243],[563,243],[564,243],[565,243],[566,243],[567,243],[568,243],[569,243],[570,243],[571,243],[572,243],[573,243],[574,243],[575,243],[576,243],[577,243],[578,243],[579,243],[580,243],[581,243],[582,243],[583,243],[584,243],[585,243],[586,243],[587,243],[588,243],[589,243],[590,243],[591,243],[592,243],[593,243],[594,243],[595,243],[596,243],[597,243],[598,243],[599,243],[600,243],[601,243],[602,243],[603,243],[604,243],[605,243],[606,243],[607,243],[608,243],[609,243],[610,243],[611,243],[612,243],[613,243],[614,243],[615,243],[616,243],[617,243],[618,243],[619,243],[620,243],[621,243],[622,242],[623,241],[440,241],[439,240],[438,239],[437,238],[436,238],[435,238],[434,238],[433,238],[432,238],[431,238],[430,237],[429,237],[428,237],[427,237],[426,237],[425,237],[424,237],[423,237],[422,237],[421,237],[420,237],[419,237],[418,237],[417,237],[416,237],[415,237],[414,237],[413,237],[412,238],[411,238],[410,238],[409,238],[408,239],[407,239],[406,239],[405,239],[404,239],[403,238],[402,238],[401,238],[400,238],[399,238],[398,238],[397,238],[396,237],[395,237],[394,237],[393,237],[392,237],[391,237],[390,237],[389,238],[388,238],[387,238],[386,238],[385,238],[384,238],[383,238],[382,238],[381,238],[380,238],[379,238],[378,238],[377,238],[376,238],[375,238],[374,238],[373,238],[372,238],[371,238],[370,238],[369,238],[368,238],[367,238],[366,238],[365,238],[364,238],[363,238],[362,238],[361,238],[360,238],[359,238],[358,238],[357,238],[356,238],[355,239],[354,239],[353,239],[352,239],[351,239],[350,240],[349,240],[348,241],[347,241],[346,241],[345,241],[344,241],[343,241],[342,240],[341,240],[340,239],[442,241],[443,240],[444,239],[444,238],[445,237],[338,239],[337,239],[336,239],[335,239],[334,239],[333,239],[332,239],[331,239],[330,239],[329,239],[328,239],[327,239],[326,239],[325,239],[324,239],[323,239],[322,239],[321,239],[320,239],[319,239],[318,239],[317,239],[316,239],[315,239],[314,239],[313,239],[312,240],[311,240],[310,240],[309,239],[308,239],[307,238],[306,238],[305,238],[304,238],[303,238],[302,237],[301,237],[300,237],[299,237],[298,237],[297,237],[296,237],[295,237],[294,237],[293,237],[292,237],[291,237],[290,237],[289,237],[288,237],[287,237],[286,237],[285,237],[284,237],[283,237],[282,237],[281,237],[280,237],[279,237],[278,237],[277,237],[276,237],[275,237],[274,237],[273,237],[272,237],[271,237],[270,237],[269,237],[268,237],[267,237],[266,237],[265,237],[264,237],[263,237],[262,237],[261,237],[260,237],[259,237],[258,237],[257,237],[256,237],[255,237],[254,237],[253,237],[252,237],[251,238],[250,238],[249,238],[248,238],[247,238],[246,238],[245,238],[244,238],[243,238],[242,238],[241,238],[240,237],[239,237],[238,236],[237,235],[236,234],[235,233],[234,232],[233,231],[232,230],[231,229],[230,229],[229,228],[228,227],[448,235],[449,235],[450,235],[451,235],[452,235],[453,235],[454,235],[455,235],[456,235],[457,235],[458,235],[459,235],[460,235],[461,235],[462,235],[463,235],[464,235],[465,235],[466,235],[467,235],[468,235],[469,235],[470,235],[471,235],[472,235],[473,235],[474,235],[475,234],[476,234],[477,234],[478,233],[479,232],[480,231],[481,231],[482,230],[483,230],[484,229],[485,229],[486,229],[487,229],[488,229],[489,229],[490,229],[491,229],[492,229],[493,229],[494,229],[495,229],[496,229],[497,229],[498,229],[499,229],[500,229],[501,229],[502,229],[503,229],[504,228],[505,228],[506,228],[507,228],[508,228],[509,228],[510,228],[511,228],[512,228],[512,226],[511,225],[510,224],[509,223],[508,222],[507,221],[506,220],[505,219],[504,218],[503,217],[502,216],[501,215],[500,214],[499,213],[498,212],[497,211],[496,210],[495,209],[495,208],[495,207],[495,206],[495,205],[495,204],[495,203],[495,202],[495,201],[495,200],[495,199],[495,198],[495,197],[495,196],[495,195],[495,194],[495,193],[495,192],[495,191],[495,190],[495,189],[495,188],[495,187],[495,186],[495,185],[495,184],[495,183],[495,182],[495,181],[495,180],[495,179],[495,178],[227,224],[227,223],[227,222],[227,221],[227,220],[227,219],[227,218],[227,217],[227,216],[227,215],[227,214],[227,213],[227,212],[227,211],[227,210],[227,209],[227,208],[227,207],[227,206],[227,205],[227,204],[227,203],[227,202],[227,201],[227,200],[227,199],[227,198],[227,197],[227,196],[227,195],[227,194],[227,193],[227,192],[227,191],[227,190],[227,189],[227,188],[227,187],[227,186],[227,185],[227,184],[227,183],[227,182],[227,181],[227,180],[227,179],[227,178],[227,177],[227,176],[227,175],[227,174],[227,173],[227,172],[227,171],[227,170],[493,175],[492,174],[491,173],[490,172],[489,172],[488,172],[487,171],[486,171],[485,171],[484,170],[483,170],[482,170],[481,170],[480,170],[479,170],[478,170],[477,170],[476,170],[475,170],[474,170],[473,170],[472,170],[471,170],[470,170],[496,175],[497,174],[498,173],[498,172],[499,171],[499,170],[499,169],[500,168],[501,167],[501,166],[502,165],[503,165],[504,164],[505,164],[506,164],[507,164],[508,164],[509,164],[510,164],[511,164],[512,164],[513,164],[514,163],[515,163],[516,163],[517,162],[518,162],[519,160],[518,159],[518,158],[518,157],[517,156],[517,155],[517,154],[517,153],[517,152],[517,151],[517,150],[517,149],[517,148],[517,147],[517,146],[517,145],[517,144],[517,143],[517,142],[517,141],[517,140],[517,139],[517,138],[517,137],[517,136],[517,135],[517,134],[517,133],[517,132],[517,131],[517,130],[517,129],[517,128],[516,127],[516,126],[515,125],[515,124],[514,123],[513,122],[512,121],[511,120],[510,119],[509,118],[508,117],[507,116],[506,115],[490,124],[491,124],[492,124],[493,123],[494,123],[495,123],[496,123],[497,122],[498,121],[499,120],[500,119],[501,118],[502,117],[503,116],[504,115],[505,113],[505,112],[505,111],[505,110],[505,109],[505,108],[505,107],[505,106],[505,105],[505,104],[505,103],[505,102],[505,100],[504,99],[504,98],[504,97],[503,96],[502,95],[501,94],[500,93],[499,92],[498,91],[497,90],[496,89],[495,89],[494,88],[493,87],[492,86],[491,85],[490,84],[490,83],[490,82],[490,81],[490,80],[490,79],[490,78],[490,77],[490,76],[490,75],[490,74],[490,73],[490,72],[490,71],[490,70],[490,69],[490,68],[491,67],[491,66],[492,65],[492,64],[493,63],[493,62],[493,61],[494,60],[494,59],[494,58],[495,57],[495,56],[495,55],[495,54],[496,53],[496,52],[497,51],[497,50],[497,49],[497,48],[498,47],[498,46],[498,45],[498,44],[498,43],[499,42],[499,41],[499,40],[499,39],[499,38],[499,37],[499,36],[499,34],[499,33],[498,32],[498,31],[497,30],[497,29],[496,28],[495,27],[495,26],[494,25],[493,24],[492,23],[492,20],[493,19],[494,18],[494,17],[494,16],[495,15],[495,14],[495,13],[496,12],[496,11],[496,10],[496,9],[496,8],[496,7],[496,6],[496,5],[496,4],[496,3]])
a = np.array([[441,257],[418,278],[423,275],[428,270],[433,265],[438,260],[459,274],[454,269],[449,264],[444,259],[624,240],[689,261],[684,260],[679,255],[674,254],[669,253],[664,251],[659,251],[654,252],[649,251],[644,249],[639,248],[634,247],[629,245],[541,244],[541,256],[541,251],[541,246],[226,226],[35,239],[40,238],[45,237],[50,236],[55,235],[60,236],[65,237],[70,239],[75,240],[80,242],[85,244],[90,247],[95,251],[100,248],[105,246],[110,244],[115,239],[120,234],[125,232],[130,235],[135,237],[140,238],[145,238],[150,238],[155,238],[160,238],[165,238],[170,238],[175,238],[180,238],[185,238],[190,238],[195,238],[200,237],[205,237],[210,236],[215,235],[220,232],[225,227],[339,238],[343,233],[622,231],[617,226],[612,221],[607,216],[602,212],[630,231],[635,226],[640,221],[645,216],[441,230],[436,225],[431,220],[428,215],[426,210],[427,205],[428,200],[430,195],[523,222],[528,217],[533,212],[538,207],[543,202],[548,201],[519,161],[543,176],[538,174],[533,173],[528,170],[525,165],[520,162],[227,169],[221,164],[216,163],[211,164],[206,166],[201,170],[196,172],[191,172],[186,172],[181,169],[176,165],[171,162],[232,164],[237,162],[242,159],[244,154],[468,169],[468,163],[488,125],[477,136],[482,131],[487,126],[511,96],[516,94],[521,96],[526,97],[500,35],[505,32],[510,31],[515,29],[520,27],[525,24],[485,20]])
# a = np.array([[441,257],[624,240],[541,244],[226,226],[339,238],[519,161],[227,169],[468,169],[488,125],[500,35],[526,236],[556,243],[571,243],[586,243],[601,243],[616,243],[426,237],[411,238],[396,237],[381,238],[366,238],[351,239],[324,239],[309,239],[294,237],[279,237],[264,237],[249,238],[234,232],[462,235],[477,234],[492,229],[507,228],[498,212],[495,197],[495,182],[227,210],[227,195],[227,180],[479,170],[506,164],[517,146],[517,131],[507,116],[504,115],[493,87],[490,72],[495,57],[499,42],[496,6]])
a = np.array([[441,257],[541,244],[581,243],[621,243],[401,238],[361,238],[299,237],[259,237],[487,229],[495,187],[226,226],[227,185],[512,121],[493,62],[491,22]])
# a = np.array([[441,257],[541,244],[339,238],[226,226],[495,177],[488,125],[500,35],[624,238],[226,169],[496,2]])
# a = np.array([[441,257],[541,244],[576,243],[611,243],[406,239],[371,238],[339,238],[304,238],[269,237],[234,232],[482,230],[495,192],[227,190],[519,161],[516,126],[491,67],[500,35],[496,2]])
# a = np.array([[66,286],[97,258],[442,261],[130,235],[165,238],[200,237],[43,235],[340,238],[305,240],[270,237],[235,233],[375,237],[410,238],[541,238],[576,237],[611,237],[482,229],[428,200],[458,203],[711,233],[675,224],[495,192],[552,192],[527,170],[227,190],[469,157],[487,125],[479,92],[499,48],[496,2]])
a = np.array([[441,257],[541,244],[576,243],[611,243],[406,239],[371,238],[339,238],[304,238],[269,237],[234,232],[482,230],[495,192],[227,190],[519,161],[516,126],[491,67],[500,35],[496,2]])
fig, ax = plt.subplots(1)
ax.imshow(img)
ax.axis('off')
for i in range(a.shape[0]):
    circ = Circle((a[i,0],a[i,1]),5)
    ax.add_patch(circ)
plt.show()