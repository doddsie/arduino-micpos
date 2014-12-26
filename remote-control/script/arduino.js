/**
 * Created by doddsie on 12/25/14.
 */


var arduinoService = angular.module('micrcApp', ['ngResource']);

arduinoService.controller('ArduinoController', ['$resource',
    function($resource){


        return $resource('http://127.0.0.1:5000/move/right', {}, {
            query: {method:'GET', params:{ }, isArray:true}
        });


    }]);