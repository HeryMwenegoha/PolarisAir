// Here are the arrays that we need for the displaying the `orrect text and/or images.
var text = new Array();			// This is the basic show/hide text for each item
var SHtext = new Array();		// This is the show/hide text for the ShowAll link
var imgs = new Array();			// This is the images to use in show/hide if used
var UseCookies = false;         // Use a cookie to keep track of which show/hide  
                                //    blocks have been shown....

// Here are the text arrays for the basic show/hide
text["Default"] = new ShowHideText("Show","Hide");
text["News_Message"] = new ShowHideText("Message","Message");
text["Archive_News"] = new ShowHideText("Show News Item","Hide News Item");
text["FAQ_Question"] = new ShowHideText("Show Question","Hide Question");
text["FAQ_Answer"] = new ShowHideText("Show Answer","Hide Answer");
text["SectionInfo"] = new ShowHideText("[Show Details]","[Hide Details]");
text["GroupInfo"] = new ShowHideText("[Show Details]","[Hide Details]");
text["GroupLinks"] = new ShowHideText("[Show Links]","[Hide Links]");
text["LinkSolutions"] = new ShowHideText("[Show Problems/Solutions]","[Hide Problems/Solutions]");
text["LinkInfo"] = new ShowHideText("[Show Details]","[Hide Details]");
text["NewLink"] = new ShowHideText("[Show New Link Form]","[Hide New Link Form]");
text["PlusMinus"] = new ShowHideText("<img src=\"/Images/Plus.gif\" border=\"0\" align=\"absmiddle\" /> ","<img src=\"/Images/Minus.gif\" border=\"0\" align=\"absmiddle\" /> ");
text["ProbHelp"] = new ShowHideText("Show Page Help", "Hide Page Help");
text["AllProbs"] = new ShowHideText("Show Problem Pane", "Hide Problem Pane");
text["Solution"] = new ShowHideText("Show Solution", "Hide Solution");
text["FinalSoln"] = new ShowHideText("Show Final Solution", "Hide Final Solution");
text["Step1"] = new ShowHideText("Show Step 1", "Hide Step 1");
text["Step2"] = new ShowHideText("Show Step 2", "Hide Step 2");
text["Step3"] = new ShowHideText("Show Step 3", "Hide Step 3");
text["Step4"] = new ShowHideText("Show Step 4", "Hide Step 4");
text["Step5"] = new ShowHideText("Show Step 5", "Hide Step 5");
text["Step6"] = new ShowHideText("Show Step 6", "Hide Step 6");
text["Step7"] = new ShowHideText("Show Step 7", "Hide Step 7");
text["Step8"] = new ShowHideText("Show Step 8", "Hide Step 8");
text["Step9"] = new ShowHideText("Show Step 9", "Hide Step 9");
text["Step10"] = new ShowHideText("Show Step 10", "Hide Step 10");
text["Thoughts"] = new ShowHideText("Show Final Thoughts", "Hide Final Thoughts");
text["Graph"] = new ShowHideText("Show Graph", "Hide Graph");
text["AltSoln"] = new ShowHideText("Show Alternate Solution", "Hide Alternate Solution");
text["UniqueCountry"] = new ShowHideText("Unique Countries","Unique Countries");
text["UniqueOrgType"] = new ShowHideText("Unique Organization Types","Unique Organization Types");
text["Orgs"] = new ShowHideText("List of Organizations","List of Organizations");
text["Usage"] = new ShowHideText("Usage Info Stats","Usage Info Stats");
text["Material"] = new ShowHideText("Material Requested","Material Requested");
text["Response"] = new ShowHideText("Response Info Stats", "Response Info Stats");
text["ShowAnswer"] = new ShowHideText("Show Answer", "Hide Answer");

text["Temp"] = new ShowHideText("Show Temp Stuff","Hide Temp Stuff");

// Here are the text arrays for use with the ShowAll links
SHtext["Default"] = new ShowHideSHText("Expand All Sections","Collapse All Sections");
SHtext["Solution"] = new ShowHideSHText("Show All Solutions","Hide All Solutions");
SHtext["Solution_Opp"] = new ShowHideSHText("Hide All Solutions", "Show All Solutions");
SHtext["Step"] = new ShowHideSHText("Show All Steps","Hide All Steps");
SHtext["Step_Opp"] = new ShowHideSHText("Hide All Steps", "Show All Steps");

// Here are the image arrays for use in the basic show/hide if used
imgs["Default"] = new ShowHideImages("/Images/Show_Hide_Closed_Arrow.gif","/Images/Show_Hide_Open_Arrow.gif");
imgs["Arrows"] = new ShowHideImages("/Images/Show_Hide_Closed_Arrow.gif","/Images/Show_Hide_Open_Arrow.gif");
imgs["PM"] = new ShowHideImages("/Images/Plus.gif","/Images/Minus.gif");

// This is the initialization function that is run onLoad();
// Note that if an InitBlock is sent this will be block will be open regardless of the cookie settings.
function ShowHideInit(InitBlock) {
	
	if (arguments.length == 0) {
		InitBlock = '';
	}

	if ( !document.getElementById ) {
		alert("This page requires a browser that is compliant with the latest DOM in order to operate correctly.  This generally means IE 5.5+ or NS 6+");
		return;
	}
		
	var oDivs = document.body.getElementsByTagName("DIV");
	var div;
	var Link;
	var Img;
	var BlockID;
	var DefDisplay = '0';
	var onText;
	var offText;
	var onImg;
	var offImg;
	
	// First make sure everything is hidden
	for (i=0; i < oDivs.length; i++ ) {
		div = oDivs.item(i);
		
		if (div.id.indexOf("Block_") != -1) {
			// Snag the ID from div.id
			BlockID = (div.id).substring(6,(div.id).length);
			
			// Try to get the associated link and image.
			Link = document.getElementById("Link_"+BlockID);
			Img = document.getElementById("Img_"+BlockID);
			
			// Find the previous display option and set the cookie if needed.
			if (!UseCookies) {
				DefDisplay = '0';
			} else if ( Get_Cookie(div.id) ) {
				DefDisplay = Get_Cookie(div.id);
			} else {
				Set_Cookie(div.id,'0','','/','','');
				DefDisplay = '0';
			}
			
			// Now set current display to old display
			div.style.display = (DefDisplay == '0' && InitBlock != BlockID) ? "none" : "block";
			
			// Determine the correct text and image
			var DisText = GetText(BlockID);
			onText = DisText.on;
			offText = DisText.off;
			
			var DisImages = GetImages(BlockID);
			onImg = DisImages.on;
			offImg = DisImages.off;
			
			// If the link exists set the text
			if (Link != null) {
				Link.innerHTML = (DefDisplay == '0' && InitBlock != BlockID) ? onText : offText;
			}
			
			// If the image exists set the source.
			if (Img != null) {
				Img.src = (DefDisplay == '0' && InitBlock != BlockID) ? onImg : offImg;
			}
		}				
	}
	
	// In case I need to do anything with this...
	// Don't do this anymore....
	// initDHTMLAPI();
	
}

// For the basic show/hide text
function ShowHideText(on,off) {
	this.on = on;
	this.off = off;
}

// For the ShowAll link text
function ShowHideSHText(exp,col) {
	this.expand = exp;
	this.collapse = col;
}

// For the images to use with the basic show/hide if used
function ShowHideImages(on,off) {
	this.on = on;
	this.off = off;
}

// Here's the function that does the basic show/hide for a div...
//
// In order for this to work the portion that is to be shown/hidden needs to
// be enclosed in a div with an id in the form Block_XXXX
// 
// The link that is used to trigger ShowHide needs to have an id in the form
// Link_XXXX.  If there is an image to change as the show/hide state is changed
// it needs to have an id in the form Img_XXXX
//
// In all of these the XXXX needs to match for each set.
//
// In the function call id = XXXX from above and type is used to determine the
// text/image that is pulled from the initialization arrays at the top of the file.
// If type is not found in the arrays then the default text/image is used.  If you
// want to default the image to some default/common image then send this as imgtype.
//
// If only one arguement is sent then it is assumed that id = type.
//
function ShowHide(type,id, imgtype) {
	
	if (arguments.length == 1) {
		type = arguments[0];
		id = arguments[0];
		imgtype = arguments[0];
	} else if (arguments.length == 2) {
		imgtype = type;
	}
	
	if ( !document.getElementById("Block_"+id) ) {
		return;
	}
	
	// Get the Div, link and image as well as the current display state
	var objDiv = document.getElementById("Block_"+id);
	var objLink = document.getElementById("Link_"+id);
	var objImg = document.getElementById("Img_"+id);
	var currentState = objDiv.style.display;
	
	// Sometimes these don't properly intialize so...
	if (currentState == "" || currentState == null) {
		currentState = "none";	
	}
	
	var onText;
	var offText;
	var onImg;
	var offImg;
	var DefDisplay;
	
	// Set the correct link text and image source
	if (text[type]) {
		onText = text[type].on;
		offText = text[type].off;
	} else {
		onText = text["Default"].on;
		offText = text["Default"].off;
	}
	
	if (imgs[imgtype]) {
		onImg = imgs[imgtype].on;
		offImg = imgs[imgtype].off;
	} else {
		onImg = imgs["Default"].on;
		offImg = imgs["Default"].off;
	}
		
	// Change the display state and set the cookie.
	DefDisplay = (currentState == "none") ? "1" : "0";
	if (UseCookies) {
		Set_Cookie(objDiv.id,DefDisplay,'','/','','');	
	}
	objDiv.style.display = (currentState == "none") ? "block" : "none";
	
	// Set the link text
	objLink.innerHTML = (currentState == "none") ? offText : onText;
	
	// If the image exists set the source.
	if (objImg != null) {
		objImg.src = (currentState == "none") ? offImg : onImg;
	}
	
	// Are we in a solution iframe and need to force a resize?
	var CurLoc = document.location.href;
	if (CurLoc.indexOf("/Solutions/") != -1 && parent.location.href != CurLoc) {
	    //alert(parent.location.href);
	    parent.ResizeSolnFrame();
	}

}

// Here is the function for use in the ShowAll link....
//
// In the function call type is used to identify only certain show/hide blocks.
// To use this simply make sure each div/link/img following the naming convention 
// in the ShowHide() documentation all contain the string given by type.  If given,
// only those with type in their id will be switched.  
//
// Sending only one argument or a zero length string for type will ShowHide() all 
// blocks on the page.
//
// To show all blocks send action == 1.  To hide all blocks send action == 0
//
// Sending no arguments assumes that all blocks are to be shown.
//
function ShowHideAll(type, action) {
	
	if (arguments.length == 1) {
		// in this case we want everything to apply the action sent....
		action = type;
		type = "";
	} else if (arguments.length == 0) {
		type = "";
		action = 1;
	}
	
	var oDivs = document.body.getElementsByTagName("DIV");
	var div;
	var Link;
	var Img;
	var BlockID;
	var onText;
	var offText;
	var onImg;
	var offImg;
	var ExpText;
	var ColText;
	var ToggleAction = (action == 0 || action == 1);
	
	// if action = 2 (hide all) or action = 3 (show all) then we don't want to 
	// toggle the action/text but we need to set the action to 0 or 1 to have
	// the rest of the method work properly...
	if (action == 2) {
		action = 0;
	} else if (action == 3) {
		action = 1;
	}
	
	for (i=0; i < oDivs.length; i++ ) {
		div = oDivs.item(i);
		
		if (div.id.indexOf("Block_") != -1) {
			
			// if we're after a specific type here make sure we've got that
			if (type != "" && div.id.indexOf(type) == -1) {
				continue;	
			}
			
			// Snag the ID from div.id
			BlockID = (div.id).substring(6,(div.id).length);
			
			// Try to get the associated link and image.
			Link = document.getElementById("Link_"+BlockID);
			Img = document.getElementById("Img_"+BlockID);
			
			// Find the previous display option and set the cookie if needed.
			Set_Cookie(div.id,action,'','/','','');
			
			// Now set current display to old display
			div.style.display = (action == 0) ? "none" : "block";
			
			// Determine the correct text and image
			var DisText = GetText(BlockID);
			onText = DisText.on;
			offText = DisText.off;
			
			var DisImages = GetImages(BlockID);
			onImg = DisImages.on;
			offImg = DisImages.off;
			
			var DisSHText = GetSHText(BlockID);
			ExpText = DisSHText.expand;
			ColText = DisSHText.collapse;
				
			
			// If the link exists set the text
			if (Link != null) {
				Link.innerHTML = (action == 0) ? onText : offText;
			}
			
			// If the image exists set the source.
			if (Img != null) {
				Img.src = (action == 0) ? onImg : offImg;
			}
		}				
	}
	
	// Now take care of the the ShowHideAll link text and action....
	Link = document.getElementById("SH"+type);
	if (Link != null && ToggleAction) {
		Link.innerHTML = (action == 0) ? ExpText : ColText;
		Link.href = "javascript:ShowHideAll('" + type + "', " + (action == 0 ? 1 : 0) + ")";
	}
	
	// Are we in a solution iframe and need to force a resize?
	var CurLoc = document.location.href;
	if (CurLoc.indexOf("/Solutions/") != -1 && parent.location.href != CurLoc) {
	    //alert(parent.location.href);
	    parent.ResizeSolnFrame();
	}

}

// This is the function that will get the text for each basic show/hide block.
function GetText(BlockID) {
	var onText;
	var offText;
	
	if (text[BlockID]) {
		onText = text[BlockID].on;
		offText = text[BlockID].off;
	} else {
		// For this case, since we only have one details "text" and many links we'll
		// need an explicit check here
		if (BlockID.indexOf("Section") != -1) {
			onText = text["SectionInfo"].on;
			offText = text["SectionInfo"].off;
		} else if (BlockID.indexOf("GroupLinks") != -1) {
			onText = text["GroupLinks"].on;
			offText = text["GroupLinks"].off;
		} else if (BlockID.indexOf("Group") != -1) {
			onText = text["GroupInfo"].on;
			offText = text["GroupInfo"].off;
		} else if (BlockID.indexOf("Link") != -1) {
			onText = text["LinkInfo"].on;
			offText = text["LinkInfo"].off;
		} else if (BlockID.indexOf("Message") != -1) {
			onText = text["News_Message"].on;
			offText = text["News_Message"].off;
		} else if (BlockID.indexOf("Quest") != -1) {
			onText = text["FAQ_Question"].on;
			offText = text["FAQ_Question"].off;
		} else if (BlockID.indexOf("Answer") != -1) {
			onText = text["FAQ_Answer"].on;
			offText = text["FAQ_Answer"].off;
		} else if (BlockID.indexOf("NewLink") != -1) {
			onText = text["NewLink"].on;
			offText = text["NewLink"].off;
		} else if (BlockID.indexOf("AdminSoln") != -1) {
			onText = text["LinkSolutions"].on;
			offText = text["LinkSolutions"].off;
		} else if (BlockID.indexOf("Soln") != -1) {
			onText = text["Solution"].on;
			offText = text["Solution"].off;
		} else if (BlockID.indexOf("ArchN") != -1) {
			onText = text["Archive_News"].on;
			offText = text["Archive_News"].off;
		} else if (BlockID.indexOf("Step1") != -1 || BlockID.indexOf("StepA1") != -1  || BlockID.indexOf("StepB1") != -1) {
			onText = text["Step1"].on;
			offText = text["Step1"].off;
		} else if (BlockID.indexOf("Step2") != -1 || BlockID.indexOf("StepA2") != -1  || BlockID.indexOf("StepB2") != -1) {
			onText = text["Step2"].on;
			offText = text["Step2"].off;
		} else if (BlockID.indexOf("Step3") != -1 || BlockID.indexOf("StepA3") != -1  || BlockID.indexOf("StepB3") != -1) {
			onText = text["Step3"].on;
			offText = text["Step3"].off;
		} else if (BlockID.indexOf("Step4") != -1 || BlockID.indexOf("StepA4") != -1  || BlockID.indexOf("StepB4") != -1) {
			onText = text["Step4"].on;
			offText = text["Step4"].off;
		} else if (BlockID.indexOf("Step5") != -1 || BlockID.indexOf("StepA5") != -1  || BlockID.indexOf("StepB5") != -1) {
			onText = text["Step5"].on;
			offText = text["Step5"].off;
        } else if (BlockID.indexOf("Step6") != -1 || BlockID.indexOf("StepA6") != -1 || BlockID.indexOf("StepB6") != -1) {
            onText = text["Step6"].on;
            offText = text["Step6"].off;
        } else if (BlockID.indexOf("Step7") != -1 || BlockID.indexOf("StepA7") != -1 || BlockID.indexOf("StepB7") != -1) {
            onText = text["Step7"].on;
            offText = text["Step7"].off;
        } else if (BlockID.indexOf("Step8") != -1 || BlockID.indexOf("StepA8") != -1 || BlockID.indexOf("StepB8") != -1) {
            onText = text["Step8"].on;
            offText = text["Step8"].off;
        } else if (BlockID.indexOf("Step9") != -1 || BlockID.indexOf("StepA9") != -1 || BlockID.indexOf("StepB9") != -1) {
            onText = text["Step9"].on;
            offText = text["Step9"].off;
        } else if (BlockID.indexOf("Step10") != -1 || BlockID.indexOf("StepA10") != -1 || BlockID.indexOf("StepB10") != -1) {
            onText = text["Step10"].on;
            offText = text["Step10"].off;
        } else if (BlockID.indexOf("Graph") != -1) {
            onText = text["Graph"].on;
            offText = text["Graph"].off;
		} else if (BlockID.indexOf("Samp1") != -1) {
			onText = text["Temp"].on;
			offText = text["Temp"].off;
		}  else if (BlockID.indexOf("Samp2") != -1) {
			onText = text["PlusMinus"].on;
			offText = text["PlusMinus"].off;
		}  else {
			onText = text["Default"].on;
			offText = text["Default"].off;
		}
	}
	
	return new ShowHideText(onText,offText);
}

// This is the function that will get the images for each basic show/hide block if used
function GetImages(BlockID) {
	var onImg;
	var offImg;
	
	if (imgs[BlockID]) {
		onImg = imgs[BlockID].on;
		offImg = imgs[BlockID].off;
	} else {
		if (BlockID.indexOf("Soln") != -1) {
			onImg = imgs["Arrows"].on;
			offImg = imgs["Arrows"].off;
		} else {
			onImg = imgs["Default"].on;
			offImg = imgs["Default"].off;
		}
	}
	
	return new ShowHideImages(onImg,offImg);
}

// This is the function that will get the text for each ShowAll link.
function GetSHText(BlockID) {
	var ExpText;
	var ColText;
	
	if (SHtext[BlockID]) {
		ExpText = SHtext[BlockID].expand;
		ColText = SHtext[BlockID].collapse;
	} else {
		if (BlockID.indexOf("Soln") != -1) {
			ExpText = SHtext["Solution"].expand;
			ColText = SHtext["Solution"].collapse
		} else if (BlockID.indexOf("Step") != -1) {
			ExpText = SHtext["Step"].expand;
			ColText = SHtext["Step"].collapse;
		} else {
			ExpText = SHtext["Default"].expand;
			ColText = SHtext["Default"].collapse;
		}
	}
	
	return new ShowHideSHText(ExpText,ColText);
}

// Here is the function that will display a basic popup.  The material to be 
// displayed in the popup should be in a div with id in the form PopUp_XXXX.  The
// area that is mousedover or clicked should be in a span/img/etc with id whose id 
// can be either S_XXXX (to have the function assume this) or some other id.  Note
// that in order for the placement of the popup to be correct this really does need
// to be found by the routine and so the id must be known.
//
// If the popup is a balloon and you want to adjust the top/side to account for
// viewing issues then you'll need to make sure there are two div's for top/bottom
// image containing the balloon tip named BalTop_XXXX and BalBot_XXXX.
//
// In the function call id == XXXX.  If the span id is not S_XXXX then the final 
// argument needs to be this id, if not there is no need to send the final arguement
// as the function will then assume S_XXXX.  Offset_Above and Offset_Below is how 
// much to offiset if displaying above/below the link.
//
// Offset_##### - These tell the function how much to offset the popup in the 
//                various directions depending upon where the popup is occuring.
//                Offset_Left is used when aligning with the left edge of calling
//                element or if we're aligning to the mouse click then it will
//                shift if aligning left edge of popup to mouse click, etc... 
//                The defaults are all zero.
// adjtop - boolean to tell the function to determine if the popup will occur 
//          outside of the viewing range and adjust accordingly.  The function
//          assumes the popup will occur above link and then will adjust to below
//          link if needed.  The default is set to true and so will adjust.
// adjside - Same thing as adjtop but for right/left.  The default is to assume
//           the popup will occur to the right and then set to left if will appear
//           out of viewing range.
// pctdel - The popup will slowly appear if that is what you want.  This determines
//          the amount that will show up each timedel.  The default is 5% of the
//          popup will show each time.  Set to 100 to have it appear completely.
// timedel - This is the time differential to use when showing the popup.
// isBalloon - A boolean variable telling the function if the popup is a balloon
//             or not as it will need to adjust accordingly if adjtop and/or
//             adjside are true.  If both of these are false then it won't matter.
//
// popW - This will set the width of the element PopUp_XXXX and if isBalloon is
//        set to true it will also set with width for BalTop_XXXX, BalHead_XXXX,
//        BalText_XXXX, and BalBot_XXXX.  Set to negative to not set width.
//
// popH - This will set the height of the element PopUp_XXXX.  Set to negative to
//        not set a width.
//
// align - Where should the popup align be default.
//         L = align to left sides of popup and calling element
//         R = align to right sides of popup and calling element
//         C = center popup over calling element
//         LR = align left edge of popup to right edge of calling element
//         RL = align right edge of popup to left edge of calling element
//         ML = align left edge of popup at mouse click
//         MR = align right edge of popup at mouse click
//         MC = aligh center of popup at mouse click
//
// horiz - Align along the top (T) or bottom (B) of the calling element.
//
// the evt argument should just be entered as event to get the mouseover/click
// event send down to this.
//
function ShowPopUp(id, evt, Offset_Above, Offset_Below, Offset_Right, Offset_Left, adjtop, adjside, pctdel, timdel, isBalloon, popW, popH, align, horiz, SpanID) {
	var initPercent;
	var leftpos;
	var rightpos;
	var toppos;
	var botpos;
	
	if ( !document.getElementById("PopUp_"+id) || arguments.length < 2 ) {
		return;
	}
	
	if (arguments.length == 2) {
	    Offset_Above = 0;
	    Offset_Below = 0;
	    Offset_Right = 0;
	    Offset_Left = 0;
	    adjtop = true;
	    adjside = true;
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 3) {
	    Offset_Below = 0;
	    Offset_Right = 0;
	    Offset_Left = 0;
	    adjtop = true;
	    adjside = true;
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 4) {
	    Offset_Right = 0;
	    Offset_Left = 0;
	    adjtop = true;
	    adjside = true;
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 5) {
	    Offset_Left = 0;
	    adjtop = true;
	    adjside = true;
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 6) {
	    adjtop = true;
	    adjside = true;
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 7) {
	    adjside = true;
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 8) {
	    pctdel = 100;
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 9) {
	    timdel = 1;
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 10) {
	    isBalloon = false;
	    popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 11) {
		popW = -1;
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 12) {
	    popH = -1;
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 13) {
	    align = "L";
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 14) {
	    horiz = "T";
		SpanID = "S_"+id;
	} else if (arguments.length == 15) {
		SpanID = "S_"+id;
	}  
	
	var objDiv = document.getElementById("PopUp_"+id);
	var currentState = objDiv.style.display;
	
	// Make sure the align arguement is upper case...
	align = align.toUpperCase();
	
	if (currentState == "none" || currentState == "") {
		
		evt = (evt) ? evt : ((window.event) ? event : null);
		
		// We need this set to "block" now to determine the correct height.
		objDiv.style.display = "block";
		
		if (popW > -1) {
		    DHTMLAPI.setWidth("PopUp_"+id, popW);
		    if (isBalloon) {
		        DHTMLAPI.setWidth("BalTop_"+id, popW);
		        DHTMLAPI.setWidth("BalHead_"+id, popW);
		        DHTMLAPI.setWidth("BalBot_"+id, popW);
		        DHTMLAPI.setWidth("BalText_"+id, popW);
		    }
		}
		
		if (popH > -1) {
		    DHTMLAPI.setHeight("PopUp_"+id, popH);
		    DHTMLAPI.setHeight("BalText_"+id, popH);
		}
		// PageC gives the location of the click in the page.
		// DivC gives the location of the click relative to the lower left 
		//      corner of the element containing the click.  I.e. it gives
		//      the location of the click inside the containing element.
		// DivW, DivH gives the width and height of the popup.
		// SpanW, SpanH gives the width and height of the element clicked
		//              to get the popup....
		var PageC = DHTMLAPI.getPageEventCoords(evt);
		var DivC = DHTMLAPI.getPositionedEventCoords(evt);
		var DivW = DHTMLAPI.getElementWidth("PopUp_"+id);
		var DivH = DHTMLAPI.getElementHeight("PopUp_"+id);
		var SpanH = DHTMLAPI.getElementHeight(SpanID);
		var SpanW = DHTMLAPI.getElementWidth(SpanID);
		var SpanC = DHTMLAPI.getElementPosition(SpanID);
		var WinW = DHTMLAPI.getInsideWindowWidth();
		var WinH = DHTMLAPI.getInsideWindowHeight();
		
		var coords = {left:0, top:0};
		
		var didAdjSide = false;
		var didAdjTop = false;
		
		// First we need to figure out where the left edge of the popup should be
		switch(align) {
		    // Right edges of popup and calling element are aligned
		    case "R":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the left edge of popup....
		        leftpos = PageC.left - (DivW - SpanW + DivC.left) + Offset_Right;
		        //leftpos = SpanC.left + SpanW - DivW + Offset_Right;
		        if ( leftpos > 0  || !adjside ) {
        		    coords.left = leftpos;
		        } else {
					// This will match up left edges instead...
		            didAdjSide = true;
		            coords.left = PageC.left - DivC.left + Offset_Left;
		            //coords.leftpos = SpanC.left + Offset_Left;
		        }
		        break;
		    // Center popup over calling element
		    case "C":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the left edge of popup....
		        leftpos = PageC.left - DivC.left + parseInt(SpanW/2) - parseInt(DivW/2);
		        rightpos = PageC.left - DivC.left + parseInt(SpanW/2) + parseInt(DivW/2);
		        //leftpos = SpanC.left + parseInt(SpanW/2) - parseInt(DivW/2);
		        //rightpos =  SpanC.left + parseInt(SpanW/2) + parseInt(DivW/2);
		        // Note that if both edges are outside the window then it's probably
		        // best that we don't adjust the position.....
		        if ( (leftpos > 0  && rightpos < WinW)  ||
		             (leftpos < 0 && rightpos > WinW) || !adjside ) {
        		    coords.left = leftpos;
		        } else if (leftpos > 0  && rightpos > WinW) {
					// We're past the right edge so line up right edges...
		            didAdjSide = true;
		            coords.left = PageC.left - (DivW - SpanW + DivC.left);
		            //coords.left = SpanC.left + SpanW - DivW;
		        } else if (leftpos < 0  && rightpos < WinW) {
					// We're past the left edge so line up left edges..
		            didAdjSide = true;
		            coords.left = PageC.left - DivC.left + Offset_Left;
		            //coords.left = SpanC.left;
		        }
		        break;
		    // Left edge of popup aligns with right edge of calling element
		    case "LR":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the right edge of popup....
		        rightpos = PageC.left + SpanW - DivC.left + DivW + Offset_Right;
		        //rightpos = SpanC.left + SpanW + DivW + Offset_Right;
		        if ( rightpos  < WinW || !adjside ) {
					//coords.left = PageC.left + SpanW - DivC.left + Offset_Right;
        		    coords.left = PageC.left + SpanW - DivC.left + Offset_Right;
		        } else {
					// we're past the right edge so align the right edges...
					coords.left = PageC.left - 
		                            (DivW - SpanW + DivC.left) + Offset_Right;
		            //coords.left = SpanC.left + SpanW - DivW + Offset_Right;
		            didAdjSide = true;
		        }
		        break;
		    // Right edge of popup aligns with left edge of calling element
		    case "RL":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the left edge of popup....
		        leftpos = PageC.left - (DivW + DivC.left) + Offset_Left;
		        //leftpos = SpanC.left - DivW + Offset_Left;
		        if ( leftpos > 0  || !adjside ) {
        		    coords.left = leftpos;
		        } else {
					// we're past the left edge so line up left edges.
		            didAdjSide = true;
		            coords.left = PageC.left - DivC.left + Offset_Left;
		            //coords.left = SpanC.left + Offset_Left;
		        }
		        break;
		    // Align left edge of popup to mouse click....
		    case "ML":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the right edge of popup....
		        rightpos = PageC.left + DivW + Offset_Left;
		        if ( rightpos < WinW || !adjside ) {
        		    coords.left = PageC.left + Offset_Left;
		        } else {
					// We're past the right edge so line up right edge with mouse.
		            didAdjSide = true;
		            coords.left = PageC.left - DivW + Offset_Right;
		            //coords.left = SpanC.left + SpanW - DivW + Offset_Right;
		        }
		        break;
		    // Align right edge of popup to mouse click....
		    case "MR":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the left edge of popup....
		        leftpos = PageC.left - DivW + Offset_Left;
		        if ( leftpos > 0  || !adjside ) {
        		    coords.left = leftpos;
		        } else {
					// We're past the left edge so line up left edge with mouse.
		            didAdjSide = true;
		            coords.left = PageC.left + Offset_Left;
		            //coords.left = SpanC.left + Offset_Left;
		        }
		        break;
		    // Center popup over mouse click
		    case "MC":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the left edge of popup....
		        leftpos = PageC.left - parseInt(DivW/2);
		        rightpos = PageC.left + parseInt(DivW/2);
		        // Note that if both edges are outside the window then it's probably
		        // best that we don't adjust the position.....
		        if ( (leftpos > 0  && rightpos < WinW)  ||
		             (leftpos < 0 && rightpos > WinW) || !adjside ) {
        		    coords.left = leftpos;
		        } else if (leftpos > 0  && rightpos > WinW) {
					// We're past the right edge line so line up the right edges.
		            didAdjSide = true;
		            coords.left = PageC.left - (DivW - SpanW + DivC.left);
		            //coords.left = SpanC.left + SpanW - DivW + Offset_Right;
		        } else if (leftpos < 0  && rightpos < WinW) {
					// We're past the left edge so line up the left edges.
		            didAdjSide = true;
		            coords.left = PageC.left - DivC.left + Offset_Left;
		            //coords.left = SpanC.left + Offset_Left;
		        }
		        break;
		    // Left edges of popup and calling element are aligned
		    case "L": default:
		        // Figure out if we need to adjust the side location or not.
		        // Here is the location of the right edge of popup....
		        rightpos = DivW - DivC.left + PageC.left + Offset_Left;
		        //rightpos = SpanC.left + DivW + Offset_Left;
		        if (rightpos  < WinW || !adjside ) {
					coords.left = PageC.left - DivC.left + Offset_Left;
        		    //coords.left = SpanC.left + Offset_Left;
		        } else {
		            didAdjSide = true;
		            coords.left = PageC.left - 
		                            (DivW - SpanW + DivC.left) + Offset_Right;
		            //coords.left = SpanC.left + SpanW - DivW + Offset_Right;
		        }
		        break;
		}
		
		// Next figure out where the top should be.....
		switch(horiz) {
		    // Right edges of popup and calling element are aligned
		    case "B":
		        // Figure out if we need to adjust the side location or not
		        // Here is the location of the left edge of popup....
		        botpos = evt.clientY + DivC.top + DivH + Offset_Below;
		        if ( botpos < WinH  || !adjtop ) {
        		    coords.top = PageC.top + (SpanH - DivC.top) + Offset_Below;
		        } else {
		            didAdjSide = true;
		            coords.top = SpanC.top - DivH + Offset_Above;
		        }
		        break;
		     // Left edges of popup and calling element are aligned
		    case "T": default:
		        // Figure out if we need to adjust the side location or not.
		        // Here is the location of the top edge of popup....
		        toppos = evt.clientY - (DivC.top + DivH) + Offset_Above;
		        if (toppos > 0 || !adjtop) {			
			        coords.top = PageC.top - (DivC.top + DivH) + Offset_Above;
		        } else {
		            didAdjTop = true;
			        coords.top = PageC.top + (SpanH - DivC.top) + Offset_Below;
		        }
		        break;
		}
		
		// Now get the popup into the correct place...
		DHTMLAPI.moveTo(objDiv.id, coords.left, coords.top);
		
		// If it was a balloon get the tip set up correctly....
		if (isBalloon) {
		    if (didAdjTop) {
		        DHTMLAPI.setDisplay("BalBot_"+id, "none");   
		        DHTMLAPI.setDisplay("BalTop_"+id, "block");
		        DHTMLAPI.setBorderWidth("BalText_"+id, "", "1px", "", "");
		        DHTMLAPI.setBorderWidth("BalHead_"+id, "0px", "", "", ""); 
		        
		        if (didAdjSide) {
		            DHTMLAPI.setBGImage("BalTop_"+id, "/Images/Balloon_Top_Right.gif", "", "right bottom");
		        } else {
		            DHTMLAPI.setBGImage("BalTop_"+id, "/Images/Balloon_Top_Left.gif", "", "left bottom");
		        }
		          
		    } else if (!didAdjTop) {
		        DHTMLAPI.setDisplay("BalBot_"+id, "block");   
		        DHTMLAPI.setDisplay("BalTop_"+id, "none"); 
		        DHTMLAPI.setBorderWidth("BalText_"+id, "", "0px", "", "");
		        DHTMLAPI.setBorderWidth("BalHead_"+id, "1px", "", "", "");
		        
		        if (didAdjSide) {
		            DHTMLAPI.setBGImage("BalBot_"+id, "/Images/Balloon_Bottom_Right.gif", "", "right top");
		        } else {
		            DHTMLAPI.setBGImage("BalBot_"+id, "/Images/Balloon_Bottom_Left.gif", "", "left top");
		        } 
		    }
		}
		
		// Show the object.....
		initPercent = (didAdjTop) ? 0 : 100;
		AddClip("PopUp_"+id, didAdjTop, initPercent, pctdel, timdel);
	} else {
	    // Remove the object...
	    // currently the didAdjTop isn't set for this and so
	    // doesn't do anything.....  I'll probably need to set a cookie
	    // in the previous bit to get this to work properly.
		initPercent = (!didAdjTop) ? 0 : 100;
		RemoveClip("PopUp_"+id, !didAdjTop, initPercent, pctdel, timdel);
	}
}

function AddClip(elemRef, FromTop, percent, pctdel, timedel) {
    var elem = DHTMLAPI.getStyleObject(elemRef);
    var clipheight;
    var height;
    var width;
    
    if (elem == null) {
        return;
    }
    //alert(elem.clip);
    // if pctdel == 100 then there's no reason to do any clipping....
    if (pctdel == 100) {
        elem.display = "block";
        return;
    }
    
    // Firefox needs the +4.  This is probably from the borders
    // and I'll need to fix this up....
	width = DHTMLAPI.getElementWidth(elemRef)+4;
	height = DHTMLAPI.getElementHeight(elemRef);
	clipheight = parseInt(percent*height/100);
	
	if (FromTop) {
		elem.clip = "rect(0px " + width + "px " + clipheight + "px 0px)";
		percent += pctdel;
		
		if (percent <= 100 ) {
			setTimeout("AddClip('" + elemRef + "', true, " + percent + ", " + pctdel + ", " + timedel + ")", timedel);	
		}
	} else {
		elem.clip = "rect(" + clipheight + "px " + width + "px " + height + "px 0px)";
		percent -= pctdel;
		
		if (percent >= 0 ) {
			setTimeout("AddClip('" + elemRef + "', false, " + percent + ", " + pctdel + ", " + timedel + ")", timedel);	
		}
		
	}
}

function RemoveClip(elemRef, FromTop, percent, pctdel, timedel) {
    var elem = DHTMLAPI.getStyleObject(elemRef);
    var clipheight;
    var height;
    var width;
    
    if (elem == null) {
        return;
    }
    
    // if pctdel == 100 then there's no reason to do any clipping....
    if (pctdel == 100) {
        elem.display = "none";
        return;
    }
    
    // Firefox needs the +4.  This is probably from the borders
    // and I'll need to fix this up....
	width = DHTMLAPI.getElementWidth(elemRef)+4;
	height = DHTMLAPI.getElementHeight(elemRef);
	clipheight = parseInt(percent*height/100);
    
	if (FromTop) {	
		elem.clip = "rect(" + clipheight + "px " + width + "px " + height + "px 0px)";
		percent += pctdel;
		
		if (percent <= 100) {
			setTimeout("RemoveClip('" + elemRef + "', true, " + percent + ", " + pctdel + ", " + timedel + ")", timedel);	
		} else {
		    // Once done with the clipping this completely removes
		    // the element from the dispaly...
			elem.display = "none";	
		}
	} else  {
		elem.clip = "rect(0px " + width + "px " + clipheight + "px 0px)";
		percent -= pctdel;
		
		if (percent >= 0) {
			setTimeout("RemoveClip('" + elemRef + "', false, " + percent + ", " + pctdel + ", " + timedel + ")", timedel);	
		} else {
		    // Once done with the clipping this completely removes
		    // the element from the dispaly...
			elem.display = "none";	
		}
	}
}





// Menu Items Here....

var MenuShowing = "";    // Used to track if a menu is currently being shown or not....

// Current list of menus.....
var Menus = ["Home", "Content", "Sections", "Download", "Misc", "Help", "Contact"];

var MenuInitialized = false;

function MenuInit() {
    
    // Hide all menus just in case one is still open for some reason....
    // We'll only do this for "newer" browsers that support this call to keep the overhead to a minimum....
    if (document.body.getElementsByClassName) {
        var oDivs = document.body.getElementsByClassName("Menu_Content");
        var div;
	
        for (i = 0; i < oDivs.length; i++) {
            div = oDivs.item(i);
            div.style.display = "none";
        }
    }

}
addOnLoadEvent(function () { MenuInit(); MenuInitialized = true; });

// Add this in to avoid some odd menu behavior if hovering over a menu item while refreshing...
if (window.addEventListener || window.attachEvent) {
    addEvent(window, "unload", function () { MenuInit(); MenuInitialized = false; }, false);
}

function Menu(evt, MenuLinkID, EventType) {
    
    // This will only be true after the "onload" event has fired and so we should be 
    // ready at that point to actually do something with the menu.  Without this the
    // menu would occasionally appear in the wrong position...
    if (!MenuInitialized) {
        return;
    }

    var elem = (evt.target) ? evt.target : evt.srcElement;
    if (EventType == "MenuItemClick") {
        
        // Only do something if we clicked on a link...
        // Wouldn't want to close the menu item if we just clicked on some blank space somewhere...
        if (elem.tagName != "A") {
            return;
        } else {
            if (elem.attributes.getNamedItem("id") != null && elem.attributes.getNamedItem("id").value.indexOf("Link_Help") != -1) {
                // We have a Show/Hide link in the Site Help menu to don't want to close the menu...
                return;
            }
        }
    }

    var MenuItemID = MenuLinkID.replace("Link", "MenuItem");
    //var MenuImgID = MenuLinkID.replace("_p", "_i");
    var MenuTitle = MenuLinkID.substring(MenuLinkID.indexOf("_p") + 2, MenuLinkID.length - 4);;
    
    var HomeMenuLinkID = MenuLinkID.replace(MenuTitle, "Home");  // Need this for placement of the menu...
    
    var objMenuLink = document.getElementById(MenuLinkID);
    var objMenuItem = document.getElementById(MenuItemID);
    var objHomeMenu = document.getElementById(HomeMenuLinkID);
    //var objImg = document.getElementById(MenuImgID);

    var isHomeMenuLink = (MenuLinkID.indexOf("Home") != -1);
    var isContactMenuLink = (MenuLinkID.indexOf("Contact") != -1);
    
    // both of these objects must exist to do pretty much anything....
    if (!objMenuLink || !objHomeMenu ) {
        return;
    }

    // This won't exist for Home/Contact links but must exist of all others....
    if (!isHomeMenuLink && !isContactMenuLink && !objMenuItem) {
        return;
    }
    
    //var onImg = "/images/Menu_Arrow_On.gif";
    //var offImg = "/images/Menu_Arrow_Off.gif";

    var onLinkCss;
    var offLinkCss;

    // Home and Contact Links have different Css Classes due to borders...
    if (isHomeMenuLink) {
        onLinkCss = "Menu_Item_Hover_Left Menu_Desktop";
        offLinkCss = "Menu_Item_Left Menu_Desktop";
    } else if (isContactMenuLink) {
        onLinkCss = "Menu_Item_Hover_Right Menu_Desktop";
        offLinkCss = "Menu_Item_Right Menu_Desktop";
    } else {
        onLinkCss = "Menu_Item_Hover Menu_Desktop";
        offLinkCss = "Menu_Item Menu_Desktop";
    }

    if (EventType == "LinkEnter") {
        // First make sure there are no "orphan" menus that somehow got left open....
        CloseAllMenus(MenuItemID, MenuTitle);
        
        // Flip CSS class to "on"
        objMenuLink.className = onLinkCss;

        // From this point on nothing to do if we entered the Home or Contact Link...
        if (isHomeMenuLink || isContactMenuLink) {
            return;
        }

        // Flip image to on ....
        //if (objImg) {
         //   objImg.src = onImg;
        //}
        
        // Show the menu.....
        ShowMenu(evt, EventType, objMenuLink, objMenuItem, objHomeMenu, MenuLinkID, MenuTitle);

        // Set MenuShowing so we have that if needed...
        MenuShowing = MenuItemID;
    } else if (EventType == "LinkLeave") {
        
        // From this point on nothing to do if we entered the Home or Contact Link...
        if (isHomeMenuLink || isContactMenuLink) {
            // Flip CSS class to "off"
            objMenuLink.className = offLinkCss;

            return;
        }

        // Hide the Menu.  The method will return true if it actually hid the menu...
        // Won't hide the menu if we got here by going out the bottom of the header...
        var bHidMenu = HideMenu(evt, EventType, objMenuLink, objMenuItem, objHomeMenu);

        if (bHidMenu) {
            // Clear MenuShowing...
            MenuShowing = "";

            // Flip CSS class to "off"
            objMenuLink.className = offLinkCss;

            // Flip image to off ....
            //if (objImg) {
            //    objImg.src = offImg;
            //}
        }

    } else if (EventType == "LinkClick") {
        // All we need to do here is open/hide menu as needed...

        // If we clicked on Home or Contact link just redirect to appropriate page...
        if (isHomeMenuLink) {
            window.location = "/";
            //return;
        }

        if (isContactMenuLink) {
            window.location = "/contact.aspx";
            //return;
        }

        if (MenuShowing != "") {
            // So we'll need to hide the menu...
            HideMenu(evt, EventType, objMenuLink, objMenuItem, objHomeMenu);

            // Clear MenuShowing...
            MenuShowing = "";
        } else {
            // We'll be wanting to show the menu...
            ShowMenu(evt, EventType, objMenuLink, objMenuItem, objHomeMenu, MenuLinkID, MenuTitle);

            // Set MenuShowing so we have that if needed...
            MenuShowing = MenuItemID;
        }

    } else if (EventType == "MenuItemClick" || "MenuItemLeave" || "MenuItemClick") {
        // So we'll need to hide the menu...  If we send in a null for the event it will simply close the menu

        HideMenu(null, EventType, objMenuLink, objMenuItem, objHomeMenu);

        // Clear MenuShowing...
        MenuShowing = "";

        // Flip CSS class to "off"
        objMenuLink.className = offLinkCss;

        // Flip image to off ....
        //if (objImg) {
        //    objImg.src = offImg;
        //}
    }
}

function ShowMenu(evt, EventType, objMenuLink, objMenuItem, objHomeMenu, MenuLinkID, MenuTitle) {

    var elem = (evt.target) ? evt.target : evt.srcElement;

    var OffSet_Left = 0;
    var OffSet_Below = 0;

    var evtCoords = DHTMLAPI.getPageEventCoords(evt);
    var coords = { left: 0, top: 0 };

    var HomePosition = DHTMLAPI.getElementPosition(objHomeMenu.id);
    var HomeHeight = DHTMLAPI.getElementHeight(objHomeMenu.id);

    coords.left = HomePosition.left + OffSet_Left;
    coords.top = HomePosition.top + HomeHeight + OffSet_Below;

    // Just in case we manage to get this far and still no object....
    if (objMenuItem == null) {
        return;
    }

    objMenuItem.style.width = GetFullMenuWidth(MenuLinkID, MenuTitle, evt);
    
    // Now get the popup into the correct place...
    DHTMLAPI.moveTo(objMenuItem.id, coords.left, coords.top);

    objMenuItem.style.display = "block";
}


function HideMenu(evt, EventType, objMenuLink, objMenuItem, objHomeMenu) {

    var bHidMenu = false;

    if (evt == null) {
        // We got here from clicking the close link in the menu itself so just close the menu and return....
        objMenuItem.style.display = "none";

        return !bHidMenu;
    }
    
    // We got here from an onmouseleave event....
    var OffSet_Left = 0;
    var OffSet_Below = 0;

    var evtCoords = DHTMLAPI.getPageEventCoords(evt);
    var coords = { left: 0, top: 0 };

    var HomePosition = DHTMLAPI.getElementPosition(objHomeMenu.id);
    var HomeHeight = DHTMLAPI.getElementHeight(objHomeMenu.id);

    // If moving the mouse slowly the coords would be to close and the menu will close when moving out the bottom
    // of the menu header and into the menu item.  This will prevent that from happening....
    OffSet_Below = -3;

    coords.left = HomePosition.left + OffSet_Left;
    coords.top = HomePosition.top + HomeHeight + OffSet_Below;
    
    // Only close the menu if we're going out the top, left or right of the menu header.
    // If we are going out the bottom we are moving into the menu and don't want it to close....
    if (evtCoords.top < coords.top) {
        objMenuItem.style.display = "none";
        bHidMenu = true;
    }

    bHidMenu = (MenuShowing == "") ? true : bHidMenu;

    return bHidMenu;
}

function CloseAllMenus(MenuItemID, MenuTitle) {
    for (var i = 0; i < Menus.length; i++) {
        NewMenu = MenuItemID.replace(MenuTitle, Menus[i]);
        obj = document.getElementById(NewMenu);
        if (obj) {
            obj.style.display = "none";
        }
    }
}


function GetFullMenuWidth(CurrentMenu, MenuTitle, evt) {
    // Don't really need the event for the computation.  However, it provides a nice way to check
    // for old IE and/or IE compatibility mode....

    var width = 0;
    var NewMenu;
    var obj;

    var OffSet = 0;

    // Get the width of each menu element....
    for (var i = 0; i < Menus.length; i++) {
        NewMenu = CurrentMenu.replace(MenuTitle, Menus[i]);
        obj = document.getElementById(NewMenu);
        if (obj) {
            width += obj.offsetWidth;
        }
    }

    // Older versions of IE and IE in compatibility mode (needed for eqns unfortunately) need to be shifted slightly....
    if (evt.offsetX && evt.layerX == null) {
        // We have old IE and it counts borders/padding in the width.  Still was off by 1 pixel however...
        OffSet = 1;
    } else {
        // Modern browser that doesn't count borders/padding in the width....
        OffSet = -16;
    }

    return width + OffSet;
}


// this function gets the cookie, if it exists
function Get_Cookie_New( check_name ) {
	// first we'll split this cookie up into name/value pairs
	// note: document.cookie only returns name=value, not the other components
	var a_all_cookies = document.cookie.split( ';' );
	var a_temp_cookie = '';
	var cookie_name = '';
	var cookie_value = '';
	var b_cookie_found = false; // set boolean t/f default f
	
	for ( i = 0; i < a_all_cookies.length; i++ )
	{
		// now we'll split apart each name=value pair
		a_temp_cookie = a_all_cookies[i].split( '=' );
		
		
		// and trim left/right whitespace while we're at it
		cookie_name = a_temp_cookie[0].replace(/^\s+|\s+$/g, '');
	
		// if the extracted name matches passed check_name
		if ( cookie_name == check_name )
		{
			b_cookie_found = true;
			// we need to handle case where cookie has no value but exists (no = sign, that is):
			if ( a_temp_cookie.length > 1 )
			{
				cookie_value = unescape( a_temp_cookie[1].replace(/^\s+|\s+$/g, '') );
			}
			// note that in cases where cookie is initialized but no value, null is returned
			return cookie_value;
			break;
		}
		a_temp_cookie = null;
		cookie_name = '';
	}
	if ( !b_cookie_found )
	{
		return null;
	}
}