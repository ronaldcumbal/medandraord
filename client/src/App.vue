<template>
  <header>
    <!-- <p v-if="currentState">Game state: {{ currentState }}</p> -->
    <div class="text-container">
      <h2 class="word" v-if="!statusMessage">
        {{ currentWord }}
      </h2>
      <VueCountdown :auto-start="false" ref="countdown" :time="time" v-slot="{ seconds }" @end="onCountdownEnd" class="countdown">
        Sekunder kvar: {{ seconds }}
      </VueCountdown>
      <br />
      <br />
      <img v-if="currentState && !statusMessage" id="cover" v-show="imageVisible" :src="`../images/${currentWord}.jpg`" />
      <br />
      <h2 v-if="statusMessage">{{ statusMessage }}</h2>
    </div>
  </header>
</template>

<script>
import { Ros, Topic, Message } from "@breq/roslib";
import VueCountdown from "@chenfengyuan/vue-countdown";

export default {
  name: "App",
  data: () => {
    return {
      currentState: "",
      currentWord: "",
      imageVisible: true,
      counting: false,
      time: 60*1000,
      statusMessage: "",
      refresh: undefined,
      intro: true,
      ros: undefined
    }
  },
  components: {
    VueCountdown
  },
  mounted() {
    this.ros = new Ros({
      url : 'ws://0.0.0.0:8080'
    });

    this.ros.on('connection', () => {
      console.log('Connected to websocket server.');
      clearInterval(this.refresh);
    });

    this.ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      this.refresh = setInterval(location.reload(), 500);
    });

    this.ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.refresh = setInterval(location.reload(), 500);
    });

    const gameStateListener = new Topic({
      ros : this.ros,
      name : '/game_state',
      messageType : 'beginner_tutorials/GameState'
    });

    gameStateListener.subscribe((data) => {
      console.log(data);
      if (data.state === "new_game" || data.state === "new_turn" || data.state === "intro_game") {
        this.currentState = data.state;
        this.currentWord = data.data;
      }
    });
  },
  watch: {
    currentWord() {
      let intro = this.intro
      if (this.currentState === "new_turn" || this.currentState === "intro_game") {
        this.statusMessage = "";
        let time = this.time;
        this.time = 1;
        setTimeout(() => {
          this.time = time;
          setTimeout(() => {
            this.$refs.countdown.start();
          }, 0)
        }, 0)

        this.imageVisible = true;

        setTimeout(() => this.imageVisible = false, 15*1000);
      }
      if (this.currentState === "new_game") {
        this.statusMessage = "";
        this.$refs.countdown.start();
      }
    },
    currentState() {
      if (this.currentState === "end_game") {
        this.statusMessage = "Spelet har tagit slut";
        this.$refs.countdown.stop();
      } 
    }
  },
  methods: {
    startCountdown() {
      this.counting = true;
    },
    onCountdownEnd() {
      this.statusMessage = "Tiden tog slut!";

      const newTurnPublisher = new Topic({
        ros : this.ros,
        name : '/time_ran_out',
        messageType : 'std_msgs/String'
      });

      const message = new Message({
        data : "Time ran out"
      });

      newTurnPublisher.publish(message);
    },
  }
}
</script>

<style>
#cover {
  object-fit: cover;
  width: 500px;
  height: 500px;
}

.word {
  text-transform: capitalize;
}

.text-container {
  font-family: Arial, Helvetica, sans-serif;
  font-size: 24pt;
  text-align: center;
}
</style>