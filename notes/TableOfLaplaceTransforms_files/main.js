
function init(action, InitBlock) {
	var CurLoc = document.location.href;
	CurLoc = CurLoc.toLowerCase();

	var ParentLoc = parent.location.href
	ParentLoc = ParentLoc.toLowerCase();

	var isClasses = (CurLoc.indexOf("/classes/") != -1);
	var isExtra = (CurLoc.indexOf("/extras/") != -1);
	
	if (arguments.length == 0) {
		action = '';
		InitBlock = '';
	} else if (arguments.length == 1) {
		InitBlock = '';
	}
	
	if (action == "ShowHide") {
		ShowHideInit(InitBlock);
	}
	
	if (action == "PrintSoln") {
		// This could be either one or both and it won't cause any issues to do both....
		ShowHideAll("Step", 1);
		ShowHideAll("Soln", 1);
	}
	
	// See if we're on a Problem/Solution page and initialize it....
	if (document.getElementById("IFSolutions")) {
		InitProblems();
		InitSolutions();
	}
	
	// We'll need to potentially hide the loading div if a soln is loading..
	// This should only run if we're loading the solution inside an iframe and so
	// the parent location will be different.  Otherwise we're printing and we don't
	// want/need this running.
	if (CurLoc.indexOf("/solutions/") != -1 && ParentLoc != CurLoc) {
	    parent.HideLoadingDiv();
	}
		
}

function SwitchDesktopMobileView(viewmode) {
    Set_Cookie("ViewMode", viewmode, "", "/");

    // reload the page...
    location.reload();
}

function tbWaterMark_Focus(tb_ID, fcClass, fcText, boolText) {
	if ( !document.getElementById ) {
		return;
	}
	
	var box = document.getElementById(tb_ID);
	
	if (box != null) {
		box.className = fcClass;
		if (boolText) {
			box.value = fcText;
		}
	}
	
}

function tbWaterMark_Blur(tb_ID, blClass, blText, boolText) {
	if ( !document.getElementById ) {
		return;
	}
	
	var box = document.getElementById(tb_ID);
	
	if (box != null && box.value.length == 0) {
		box.className = blClass;
		if (boolText) {
			box.value = blText;
		}
	}
}


function Trim(str)
{  while(str.charAt(0) == (" ") )
  {  str = str.substring(1);
  }
  while(str.charAt(str.length-1) == " " )
  {  str = str.substring(0,str.length-1);
  }
  return str;
}

function GoToPage(URL) {
	location.href = URL;
}

function SendFile(FileURL, FileName) {
	var URL;

	// This will bring the window to the front if it already exists and 
	// has been hidden by another window.....
	window.focus();
	
	if (FileURL != "" && FileName != "") {
		URL = "/dodownload.aspx?U=";
		URL += FileURL;
		URL += "&N=";
		URL += FileName;

		location.href = URL;
	}
}

function DownloadFile(ID) {
	var URL;
	var IDParts = ID.split(';');
	var strTmp;

	var gaEventCat = "Content Page";
	
	URL = "/getfile.aspx?file=" + IDParts[0];
	
	// If I send a second part this will be the "URL" that I send
	// to Google Anayltics to help count the number of downloads so
	// send the "download" to Google....
	if (IDParts.length > 1) {
	    //_gaq.push(['_trackEvent', 'Downloads', 'PDF', IDParts[1]]);
	    strTmp = IDParts[0].split(',');

        if (strTmp.length == 3) {
            switch (strTmp[2]) {
                case "P":
                    gaEventCat += " (Practice Problems - ";
                    break;

                case "S":
                    gaEventCat += " (Practice Problems Solutions - ";
                    break;

                case "A":
                    gaEventCat += " (Assignment Problems - ";
                    break;

                default:
                    gaEventCat += " (";
                    break;
            }

            switch (strTmp[0]) {
                case "S":
                    gaEventCat += "Section)";
                    break;

                case "C":
                    gaEventCat += "Chapter)";
                    break;

                case "B":
                    gaEventCat += "Book)";
                    break;

                default:
                    gaEventCat += "";
                    break;
            }
        }

	    ga('send', 'event', 'Downloads', gaEventCat, IDParts[1]);
	}

    // Now download the file.....
	location.href = URL;
}

function ShowPage(page) {
	var target = 'Show_Win';
	var url;
	var show_win;

	var winopts = 'height=600';
	winopts += ',width=950';
	winopts += ',menubar=no';
	winopts += ',status=no';
	winopts += ',toolbar=no';
	winopts += ',location=no';
	winopts += ',scrollbars=yes';
	winopts += ',resizable=yes';

	switch (page) {
		case "NHelp":
			url = "/help/Help_Notes.aspx";
			break;
		case "PPHelp":
			url = "/help/Help_PracticeProbs.aspx";
			break;
		case "PPIHelp":
			url = "/help/Help_PracticeProbsIntro.aspx";
			break;
		case "APHelp":
			url = "/help/Help_AsgnProbs.aspx";
			break;

		default:
			url = '';
			break;
	}

	if (show_win == null) {
		show_win = window.open(url, target, winopts);
	} else {
		show_win.location.href = url;
	}
} // ShowPage

function HighlightSelection(ID, NUM, Sel) {
	
	if (arguments.length != 3) {
		return;
	}
		
	if ( !document.getElementById ) {
		return;
	}
	
	var Sel_ID;
	var Sel_Item;
	
	for (var i=1; i <= NUM; i++) {
		Sel_ID = ID + "_" + i;
		Sel_Item = document.getElementById(Sel_ID)
		
		if (Sel_Item == null) {
			continue;	
		}
		
		if (i == Sel) {
			Sel_Item.className = "SelectedItem";
		} else {
			Sel_Item.className = "";
		}
	}
} // HighlightSelection

function ToggleSelection(ID,Sel) {
	
	if (arguments.length != 2) {
		return;
	}
		
	if ( !document.getElementById ) {
		return;
	}
	
	var Sel_ID = ID+"_"+Sel;
	var Sel_IDCB = "cbl"+Sel_ID;
	
	var Sel_Item = document.getElementById(Sel_ID);
	var Sel_ItemCB = document.getElementById(Sel_IDCB);
	
	if (Sel_Item == null || Sel_ItemCB == null) {
		return;	
	}
	//alert(Sel_ItemCB.checked);
	// It won't actually "check" until after this so....
	if (Sel_ItemCB.checked) {
		Sel_Item.className = "";
	} else {
		Sel_Item.className = "SelectedItem";
	}
	
} // ToggleSelection


function Set_Cookie(name, value, expires, path, domain, secure) {
    // set time, it's in milliseconds
    var today = new Date();
    today.setTime(today.getTime());

    /*
	if the expires variable is set, make the correct 
	expires time, the current script below will set 
	it for x number of days, to make it for hours, 
	delete * 24, for minutes, delete * 60 * 24
	*/
    if (expires) {
        expires = expires * 1000 * 60 * 60 * 24;
    }
    var expires_date = new Date(today.getTime() + (expires));

    document.cookie = name + "=" + escape(value) +
	((expires) ? ";expires=" + expires_date.toGMTString() : "") +
	((path) ? ";path=" + path : "") +
	((domain) ? ";domain=" + domain : "") +
	((secure) ? ";secure" : "");
}

function Get_Cookie(cname) {
    var name = cname + "=";
    var ca = document.cookie.split(';');
    for (var i = 0; i < ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0) == ' ') c = c.substring(1);
        if (c.indexOf(name) == 0) return c.substring(name.length, c.length);
    }
    return "";
}