
"use strict";
$(document).ready(function () {
    // sets a function that will be called when the websocket connects/disconnects
    NetworkTables.addWsConnectionListener(onNetworkTablesConnection, true);

    // sets a function that will be called when the robot connects/disconnects
    NetworkTables.addRobotConnectionListener(onRobotConnection, true);

    // sets a function that will be called when any NetworkTables key/value changes
    NetworkTables.addGlobalListener(onValueChanged, true);

    //attachSelectToSendableChooser("#autonomous", "Autonomous Mode");
    NetworkTables.addKeyListener("/SmartDashboard/Auto", function(key, value, isNew){
        $('#autonomous').text(value);
    }, true);

    $("#cameraImg").attr("src", "http://10.48.18.11:5801/?action=stream");
});

function onRobotConnection(connected) {
    $('#robotstate').text(connected ? "Connected" : "Disconnected");
    $('#robotAddress').text(connected ? NetworkTables.getRobotAddress() : "disconnected");

    document.title = connected ? "Connected @ " + NetworkTables.getRobotAddress() : "Disconnected"
}

function onNetworkTablesConnection(connected) {
    if (connected) {
        $("#connectstate").text("Connected");

        // clear the table
        $("#networktables").empty();

    } else {
        $("#connectstate").text("Disconnected!");
    }
}

function onValueChanged(key, value, isNew) {
    
    var sidebarKey = key;
    if (sidebarKey[0] == '/') {
        sidebarKey = sidebarKey.substring(1);
    }
    var paths = sidebarKey.split('/');
    updateSidebar('ntSidebar', 0, paths, value);
}

function fixPath(paths) {
    // dumb way to brute force this for now
    paths.forEach(function(part, index, theArray) {
        theArray[index] = theArray[index].replace(' ', '_').replace(' ', '_').replace(' ', '_').replace(' ', '_').replace(' ', '_');
    });
    return paths;
}

function updateSidebar(parentId, index, paths, value) {

    var currentName = paths[index];
    var currentPaths = paths.slice(0, index + 1);
    var currentPath = currentPaths.join('-');
    var keyName = paths[paths.length - 1];

    var headerId = NetworkTables.keySelector(fixPath(currentPaths).join('-'));
    var id = NetworkTables.keySelector(fixPath(paths).join('-'));

    var card = $('#card' + headerId);
    var cardTableBody = $('#cardTableBody' + headerId);
    var cardCollapse = $('#collapse' + headerId);

    if(paths.length - index > 1)
    {
        // create the collapsable header
        if (card.length == 0) {
            // full container
            card = $('<div/>')
                .attr('id', 'card' + headerId)
                .attr('class', 'card text-white bg-secondary');
            // header container
            var header = $('<div/>')
                .attr('id', 'header' + headerId)
                .attr('class', 'card-header')
                .attr('style', 'padding:10px !important;')
                .appendTo(card);
            // header
            var headerH5 = $('<h5/>')
                .attr('class', 'mb-0')
                .appendTo(header);
            // button
            var headerButton = $('<button/>')
                .attr('data-toggle', 'collapse')
                .attr('data-target', '#collapse' + headerId)
                .attr('aria-expanded', 'true')
                .attr('aria-control', 'collapse' + headerId)
                .attr('class', 'btn btn-primary btn-block')
                .text(currentName)
                .appendTo(headerH5);
            //collapseable content
            cardCollapse = $('<div/>')
                .attr('aria-labelledby', 'header' + headerId)
                .attr('data-parent', '#' + parentId)
                .attr('id', 'collapse' + headerId)
                .attr('class', 'collapse')
                .appendTo(card);

            var cardTable = $('<table/>')
                .attr('class', 'table table-striped table-dark')
                .appendTo(cardCollapse);
            cardTableBody = $('<tbody/>')
                .attr('id', 'cardTableBody' + headerId)
                .appendTo(cardTable);
            // add to full accordian
            $('#'+parentId).append(card);
        }
    }

    if(paths.length - index > 2)
    {
        updateSidebar('collapse' + headerId, index + 1, paths, value)
    }

    if(paths.length - index <= 2) {
        // create the entry for this value
        var cardBody = $('#card' + id);
        if (cardBody.length == 0) {
            var tr = $('<tr/>').appendTo(cardTableBody);
            // label
            $('<td/>').text(keyName).appendTo(tr);
            // value
            cardBody = $('<td/>')
                .attr('id', 'card' + id)
                .attr('class', 'card-body')
                .text(value)
                .appendTo(tr);
        }
        else {
            // update the value
            cardBody.text(value);
        }
    }
}
