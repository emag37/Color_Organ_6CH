#ifndef HANN_WINDOW_
#define HANN_WINDOW_
#include "../Drivers/CMSIS/Include/arm_math.h"
const float32_t hannWindow[] = {	
0,
3.77965772740962e-05,
0.000151180594771427,
0.000340134910380874,
0.000604630956796859,
0.000944628745838338,
0.00136007687449446,
0.00185091253269609,
0.00241706151281168,
0.00305843822086654,
0.00377494568948339,
0.00456647559254264,
0.00543290826155973,
0.00637411270377730,
0.00738994662196968,
0.00848025643595607,
0.00964487730581998,
0.0108836331568305,
0.0121963367060627,
0.0135827894907121,
0.0150427818980994,
0.0165760931973612,
0.0181824915728215,
0.0198617341590390,
0.0216135670775249,
0.0234377254751261,
0.0253339335640674,
0.0273019046636466,
0.0293413412435765,
0.0314519349689681,
0.0336333667469460,
0.0358853067748912,
0.0382074145903025,
0.0405993391222701,
0.0430607187445522,
0.0455911813302485,
0.0481903443080604,
0.0508578147201305,
0.0535931892814526,
0.0563960544408427,
0.0592659864434626,
0.0622025513948853,
0.0652053053266945,
0.0682737942636061,
0.0714075542921032,
0.0746061116305734,
0.0778689827009385,
0.0811956742017641,
0.0845856831828408,
0.0880384971212229,
0.0915535939987150,
0.0951304423807943,
0.0987685014969554,
0.102467221322469,
0.106226042661535,
0.110044397231829,
0.113921707750418,
0.117857388021034,
0.121850843022703,
0.125901468999704,
0.130008653552846,
0.134171775732054,
0.138390206130253,
0.142663306978520,
0.146990432242509,
0.151370927720124,
0.155804131140420,
0.160289372263735,
0.164825972983019,
0.169413247426353,
0.174050502060644,
0.178737035796480,
0.183472140094124,
0.188255099070633,
0.193085189608094,
0.197961681462944,
0.202883837376380,
0.207850913185816,
0.212862157937393,
0.217916813999513,
0.223014117177384,
0.228153296828549,
0.233333575979408,
0.238554171442674,
0.243814293935788,
0.249113148200246,
0.254449933121827,
0.259823841851719,
0.265234061928494,
0.270679775400947,
0.276160158951759,
0.281674384021968,
0.287221616936238,
0.292801019028898,
0.298411746770740,
0.304052951896545,
0.309723781533331,
0.315423378329296,
0.321150880583437,
0.326905422375828,
0.332686133698534,
0.338492140587147,
0.344322565252915,
0.350176526215451,
0.356053138436005,
0.361951513451265,
0.367870759507683,
0.373809981696295,
0.379768282088018,
0.385744759869409,
0.391738511478851,
0.397748630743159,
0.403774209014585,
0.409814335308190,
0.415868096439573,
0.421934577162933,
0.428012860309440,
0.434102026925899,
0.440201156413683,
0.446309326667918,
0.452425614216887,
0.458549094361650,
0.464678841315845,
0.470813928345655,
0.476953427909915,
0.483096411800347,
0.489241951281889,
0.495389117233110,
0.501536980286678,
0.507684610969869,
0.513831079845091,
0.519975457650400,
0.526116815439995,
0.532254224724658,
0.538386757612132,
0.544513486947405,
0.550633486452881,
0.556745830868423,
0.562849596091240,
0.568943859315596,
0.575027699172326,
0.581100195868139,
0.587160431324672,
0.593207489317293,
0.599240455613625,
0.605258418111759,
0.611260466978157,
0.617245694785205,
0.623213196648401,
0.629162070363163,
0.635091416541232,
0.641000338746643,
0.646887943631258,
0.652753341069825,
0.658595644294553,
0.664413970029181,
0.670207438622517,
0.675975174181427,
0.681716304703260,
0.687429962207681,
0.693115282867903,
0.698771407141278,
0.704397479899253,
0.709992650556654,
0.715556073200279,
0.721086906716794,
0.726584314919893,
0.732047466676720,
0.737475536033525,
0.742867702340537,
0.748223150376032,
0.753541070469590,
0.758820658624500,
0.764061116639314,
0.769261652228528,
0.774421479142360,
0.779539817285623,
0.784615892835666,
0.789648938359360,
0.794638192929131,
0.799582902237993,
0.804482318713598,
0.809335701631252,
0.814142317225904,
0.818901438803083,
0.823612346848765,
0.828274329138148,
0.832886680843338,
0.837448704639903,
0.841959710812305,
0.846419017358170,
0.850825950091399,
0.855179842744098,
0.859480037067308,
0.863725882930520,
0.867916738419968,
0.872051969935680,
0.876130952287266,
0.880153068788438,
0.884117711350248,
0.888024280573021,
0.891872185836974,
0.895660845391512,
0.899389686443182,
0.903058145242268,
0.906665667168023,
0.910211706812523,
0.913695728063121,
0.917117204183505,
0.920475617893327,
0.923770461446416,
0.927001236707534,
0.930167455227694,
0.933268638318005,
0.936304317122042,
0.939274032686730,
0.942177336031735,
0.945013788217338,
0.947782960410804,
0.950484433951210,
0.953117800412740,
0.955682661666441,
0.958178629940405,
0.960605327878401,
0.962962388596925,
0.965249455740667,
0.967466183536386,
0.969612236845189,
0.971687291213196,
0.973691032920598,
0.975623159029080,
0.977483377427628,
0.979271406876687,
0.980986977050686,
0.982629828578900,
0.984199713084672,
0.985696393222957,
0.987119642716209,
0.988469246388591,
0.989745000198504,
0.990946711269438,
0.992074197919133,
0.993127289687042,
0.994105827360110,
0.995009662996835,
0.995838659949645,
0.996592692885550,
0.997271647805093,
0.997875422059586,
0.998403924366628,
0.998857074823906,
0.999234804921275,
0.999537057551115,
0.999763787016967,
0.999914959040440,
0.999990550766393,
0.999990550766393,
0.999914959040440,
0.999763787016967,
0.999537057551115,
0.999234804921275,
0.998857074823906,
0.998403924366628,
0.997875422059586,
0.997271647805093,
0.996592692885550,
0.995838659949645,
0.995009662996835,
0.994105827360110,
0.993127289687042,
0.992074197919133,
0.990946711269438,
0.989745000198504,
0.988469246388591,
0.987119642716209,
0.985696393222957,
0.984199713084672,
0.982629828578900,
0.980986977050686,
0.979271406876687,
0.977483377427628,
0.975623159029080,
0.973691032920598,
0.971687291213196,
0.969612236845189,
0.967466183536386,
0.965249455740667,
0.962962388596925,
0.960605327878401,
0.958178629940405,
0.955682661666441,
0.953117800412740,
0.950484433951210,
0.947782960410804,
0.945013788217338,
0.942177336031735,
0.939274032686730,
0.936304317122042,
0.933268638318005,
0.930167455227694,
0.927001236707534,
0.923770461446416,
0.920475617893327,
0.917117204183505,
0.913695728063121,
0.910211706812523,
0.906665667168023,
0.903058145242268,
0.899389686443182,
0.895660845391512,
0.891872185836974,
0.888024280573021,
0.884117711350248,
0.880153068788438,
0.876130952287266,
0.872051969935680,
0.867916738419968,
0.863725882930520,
0.859480037067308,
0.855179842744098,
0.850825950091399,
0.846419017358170,
0.841959710812305,
0.837448704639903,
0.832886680843338,
0.828274329138148,
0.823612346848765,
0.818901438803083,
0.814142317225904,
0.809335701631252,
0.804482318713598,
0.799582902237993,
0.794638192929131,
0.789648938359360,
0.784615892835666,
0.779539817285623,
0.774421479142360,
0.769261652228528,
0.764061116639314,
0.758820658624500,
0.753541070469590,
0.748223150376032,
0.742867702340537,
0.737475536033525,
0.732047466676720,
0.726584314919893,
0.721086906716794,
0.715556073200279,
0.709992650556654,
0.704397479899253,
0.698771407141278,
0.693115282867903,
0.687429962207681,
0.681716304703260,
0.675975174181427,
0.670207438622517,
0.664413970029181,
0.658595644294553,
0.652753341069825,
0.646887943631258,
0.641000338746643,
0.635091416541232,
0.629162070363163,
0.623213196648401,
0.617245694785205,
0.611260466978157,
0.605258418111759,
0.599240455613625,
0.593207489317293,
0.587160431324672,
0.581100195868139,
0.575027699172326,
0.568943859315596,
0.562849596091240,
0.556745830868423,
0.550633486452881,
0.544513486947405,
0.538386757612132,
0.532254224724658,
0.526116815439995,
0.519975457650400,
0.513831079845091,
0.507684610969869,
0.501536980286678,
0.495389117233110,
0.489241951281889,
0.483096411800347,
0.476953427909915,
0.470813928345655,
0.464678841315845,
0.458549094361650,
0.452425614216887,
0.446309326667918,
0.440201156413683,
0.434102026925899,
0.428012860309440,
0.421934577162933,
0.415868096439573,
0.409814335308190,
0.403774209014585,
0.397748630743159,
0.391738511478851,
0.385744759869409,
0.379768282088018,
0.373809981696295,
0.367870759507683,
0.361951513451265,
0.356053138436005,
0.350176526215451,
0.344322565252915,
0.338492140587147,
0.332686133698534,
0.326905422375828,
0.321150880583437,
0.315423378329296,
0.309723781533331,
0.304052951896545,
0.298411746770740,
0.292801019028898,
0.287221616936238,
0.281674384021968,
0.276160158951759,
0.270679775400947,
0.265234061928494,
0.259823841851719,
0.254449933121827,
0.249113148200246,
0.243814293935788,
0.238554171442674,
0.233333575979408,
0.228153296828549,
0.223014117177384,
0.217916813999513,
0.212862157937393,
0.207850913185816,
0.202883837376380,
0.197961681462944,
0.193085189608094,
0.188255099070633,
0.183472140094124,
0.178737035796480,
0.174050502060644,
0.169413247426353,
0.164825972983019,
0.160289372263735,
0.155804131140420,
0.151370927720124,
0.146990432242509,
0.142663306978520,
0.138390206130253,
0.134171775732054,
0.130008653552846,
0.125901468999704,
0.121850843022703,
0.117857388021034,
0.113921707750418,
0.110044397231829,
0.106226042661535,
0.102467221322469,
0.0987685014969554,
0.0951304423807943,
0.0915535939987150,
0.0880384971212229,
0.0845856831828408,
0.0811956742017641,
0.0778689827009385,
0.0746061116305734,
0.0714075542921032,
0.0682737942636061,
0.0652053053266945,
0.0622025513948853,
0.0592659864434626,
0.0563960544408427,
0.0535931892814526,
0.0508578147201305,
0.0481903443080604,
0.0455911813302485,
0.0430607187445522,
0.0405993391222701,
0.0382074145903025,
0.0358853067748912,
0.0336333667469460,
0.0314519349689681,
0.0293413412435765,
0.0273019046636466,
0.0253339335640674,
0.0234377254751261,
0.0216135670775249,
0.0198617341590390,
0.0181824915728215,
0.0165760931973612,
0.0150427818980994,
0.0135827894907121,
0.0121963367060627,
0.0108836331568305,
0.00964487730581998,
0.00848025643595607,
0.00738994662196968,
0.00637411270377730,
0.00543290826155973,
0.00456647559254264,
0.00377494568948339,
0.00305843822086654,
0.00241706151281168,
0.00185091253269609,
0.00136007687449446,
0.000944628745838338,
0.000604630956796859,
0.000340134910380874,
0.000151180594771427,
3.77965772740962e-05,
0
};
#endif
