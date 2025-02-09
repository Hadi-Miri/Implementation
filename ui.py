package com.vighnesh.restlinks
 
import android.content.Intent

import android.os.Bundle

import android.widget.Button

import android.widget.ImageView

import android.widget.TextView

import androidx.activity.enableEdgeToEdge

import androidx.appcompat.app.AppCompatActivity

import org.java_websocket.client.WebSocketClient

import org.java_websocket.handshake.ServerHandshake

import org.json.JSONObject

import java.net.URI
 
class LvAcceptance : AppCompatActivity() {
 
    private lateinit var webSocketClient: WebSocketClient

    private lateinit var carPointer: ImageView

    private lateinit var lvPointer: ImageView

    private lateinit var leavePlatoonButton: Button
 
    override fun onCreate(savedInstanceState: Bundle?) {

        super.onCreate(savedInstanceState)

        enableEdgeToEdge()

        setContentView(R.layout.activity_lv_acceptance)
 
        // Mock data for the TextViews

        findViewById<TextView>(R.id.platoonNameTextView).text = getString(R.string.platoon_alpha)

        findViewById<TextView>(R.id.driverNameTextView).text = getString(R.string.driver_alice_smith)

        findViewById<TextView>(R.id.numberPlateTextView).text = getString(R.string.number_plate_xyz789)

        findViewById<TextView>(R.id.destinationTextView).text = getString(R.string.destination_kronach)

        findViewById<TextView>(R.id.notificationTextView).text = getString(R.string.notification_drive_safe_and_follow_the_rules)
 
        // References to the car pointers

        carPointer = findViewById(R.id.carPointer)

        lvPointer = findViewById(R.id.lvPointer)
 
        // Initialize WebSocket and subscribe to topics

        initializeWebSocket()
 
        // Reference to the Leave Platoon button

        leavePlatoonButton = findViewById(R.id.leavePlatoonButton)
 
        // Set click listener for the button

        leavePlatoonButton.setOnClickListener {

            sendEmergencyStopMessage()

        }

    }
 
    private fun initializeWebSocket() {

        val uri = URI(Config.ROSBRIDGE_URL)

        webSocketClient = object : WebSocketClient(uri) {

            override fun onOpen(handshakedata: ServerHandshake?) {

                println("WebSocket Connected")

                subscribeToTopics()

            }
 
            override fun onMessage(message: String?) {

                message?.let {

                    handleIncomingMessage(it)

                }

            }
 
            override fun onClose(code: Int, reason: String?, remote: Boolean) {

                println("WebSocket Closed: $reason")

            }
 
            override fun onError(ex: Exception?) {

                println("WebSocket Error: ${ex?.message}")

            }

        }

        webSocketClient.connect()

    }
 
    private fun subscribeToTopics() {

        val egoPoseSubscription = """

            {

                "op": "subscribe",

                "topic": "/ego_pose"

            }

        """.trimIndent()
 
        val lvPoseSubscription = """

            {

                "op": "subscribe",

                "topic": "/lv_pose"

            }

        """.trimIndent()
 
        if (webSocketClient.isOpen) {

            webSocketClient.send(egoPoseSubscription)

            println("Subscribed to /ego_pose")
 
            webSocketClient.send(lvPoseSubscription)

            println("Subscribed to /lv_pose")

        }

    }
 
    private fun handleIncomingMessage(message: String) {

        try {

            val jsonObject = JSONObject(message)

            if (jsonObject.getString("op") == "publish") {

                when (jsonObject.getString("topic")) {

                    "/ego_pose" -> {

                        val pose = jsonObject.getJSONObject("msg")

                        val position = pose.getJSONObject("pose").getJSONObject("position")

                        val x = position.getDouble("x")

                        val y = position.getDouble("y")
 
                        // Update car pointer's position on the map

                        runOnUiThread {

                            updatePointerPosition(carPointer, x, y)

                        }

                    }
 
                    "/lv_pose" -> {

                        val pose = jsonObject.getJSONObject("msg")

                        val position = pose.getJSONObject("pose").getJSONObject("position")

                        val x = position.getDouble("x")

                        val y = position.getDouble("y")
 
                        // Update LV pointer's position on the map

                        runOnUiThread {

                            updatePointerPosition(lvPointer, x, y)

                        }

                    }

                }

            }

        } catch (e: Exception) {

            println("Error parsing message: ${e.message}")

        }

    }
 
    private fun updatePointerPosition(pointer: ImageView, x: Double, y: Double) {

        // Assuming your map's coordinates are scaled in a way that matches the x, y values

        val mapWidth = findViewById<ImageView>(R.id.sampleMapImage).width

        val mapHeight = findViewById<ImageView>(R.id.sampleMapImage).height
 
        // Calculate the position of the pointer on the map

        val pointerX = (((y / 100.0) * mapWidth) * 12) + 50 // Scaling factor (change as needed)

        val pointerY = (((x / 100.0) * mapHeight) * 12) + 50 // Scaling factor (change as needed)
 
        // Update the pointer's position on the map

        pointer.x = pointerX.toFloat()

        pointer.y = pointerY.toFloat()
 
        // Make the pointer visible

        pointer.visibility = ImageView.VISIBLE

    }
 
    private fun sendEmergencyStopMessage() {

        val emergencyStopMessage = """

            {

                "op": "publish",

                "topic": "/is_emergency_stop",

                "msg": {

                    "data": true

                }

            }

        """.trimIndent()

        if (webSocketClient.isOpen) {

            webSocketClient.send(emergencyStopMessage)

            println("Emergency Stop Published")
 
            // Navigate to Thankyou.kt

            val intent = Intent(this, Thankyou::class.java)

            startActivity(intent)

            finish()

        }

    }
 
    override fun onDestroy() {

        super.onDestroy()

        if (::webSocketClient.isInitialized) {

            webSocketClient.close()

        }

    }

}

 
