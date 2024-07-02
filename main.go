package main

// Welcome to fdsfdsfsd
// __________         __    __  .__                               __
// \______   \_____ _/  |__/  |_|  |   ____   ______ ____ _____  |  | __ ____
//  |    |  _/\__  \\   __\   __\  | _/ __ \ /  ___//    \\__  \ |  |/ // __ \
//  |    |   \ / __ \|  |  |  | |  |_\  ___/ \___ \|   |  \/ __ \|    <\  ___/
//  |________/(______/__|  |__| |____/\_____>______>___|__(______/__|__\\_____>
//
// This file can be a nice home for your Battlesnake logic and helper functions.
//
// To get you started we've included code to prevent your Battlesnake from moving backwards.
// For more info see https://docs.battlesnake.com/
// hi there
import (
	"log"
	"math"
	"math/rand"
)

func ManhattanDistance(p1, p2 Coord) int {
	return int(math.Abs(float64(p1.X-p2.X)) + math.Abs(float64(p1.Y-p2.Y)))
}

func FindClosestFood(start Coord, foodPoints []Coord) Coord {
	if len(foodPoints) == 0 {
		return Coord{} // Returning a default Point if the foodPoints list is empty
	}

	closest := foodPoints[0]
	minDistance := ManhattanDistance(start, closest)

	for _, food := range foodPoints[1:] {
		distance := ManhattanDistance(start, food)
		if distance < minDistance {
			closest = food
			minDistance = distance
		}
	}

	return closest
}

// info is called when you create your Battlesnake on play.battlesnake.com
// and controls your Battlesnake's appearance
// TIP: If you open your Battlesnake URL in a browser you should see this data
func info() BattlesnakeInfoResponse {
	log.Println("INFO")

	return BattlesnakeInfoResponse{
		APIVersion: "1",
		Author:     "crazycat911", // TODO: Your Battlesnake username
		Color:      "#07e3a5",     // TODO: Choose color
		Head:       "gamer",       // TODO: Choose head
		Tail:       "coffee",      // TODO: Choose tail
	}
}

// start is called when your Battlesnake begins a game
func start(state GameState) {
	log.Println("GAME START")
}

// end is called when your Battlesnake finishes a game
func end(state GameState) {
	log.Printf("GAME OVER\n\n")
}

func detect_danger(state GameState) (dangerZones []Coord, potentialHeadToHeads []Coord) {
	mySnake := state.You
	snakes := state.Board.Snakes

	for _, snake := range snakes {
		for _, bodypart := range snake.Body {
			if bodypart == snake.Body[snake.Length-1] {
				if snake.Health == 100 {
					dangerZones = append(dangerZones, bodypart)
				}
			} else {
				dangerZones = append(dangerZones, bodypart)
			}
		}

		if !(snake.ID == mySnake.ID) {
			if snake.Length >= mySnake.Length {
				potentialHeadToHeads = append(potentialHeadToHeads, Coord{X: snake.Head.X + 1, Y: snake.Head.Y})
				potentialHeadToHeads = append(potentialHeadToHeads, Coord{X: snake.Head.X - 1, Y: snake.Head.Y})
				potentialHeadToHeads = append(potentialHeadToHeads, Coord{X: snake.Head.X, Y: snake.Head.Y + 1})
				potentialHeadToHeads = append(potentialHeadToHeads, Coord{X: snake.Head.X, Y: snake.Head.Y - 1})
			}
		}
	}
	return dangerZones, potentialHeadToHeads
}

// move is called on every turn and returns your next move
// Valid moves are "up", "down", "left", or "right"
// See https://docs.battlesnake.com/api/example-move for available data
func move(state GameState) BattlesnakeMoveResponse {
	myHead := state.You.Body[0] // Coordinates of your head

	isMoveSafe := map[string]bool{
		"up":    true,
		"down":  true,
		"left":  true,
		"right": true,
	}

	if myHead.X == 0 {
		isMoveSafe["left"] = false
	} else if myHead.X == state.Board.Width-1 {
		isMoveSafe["right"] = false
	}

	if myHead.Y == 0 {
		isMoveSafe["down"] = false
	} else if myHead.Y == state.Board.Height-1 {
		isMoveSafe["up"] = false
	}

	dangerZones, potentialHeadToHeads := detect_danger(state)

	for _, dangerZone := range dangerZones {
		if (Coord{X: myHead.X + 1, Y: myHead.Y}) == dangerZone {
			isMoveSafe["right"] = false
		} else if (Coord{X: myHead.X - 1, Y: myHead.Y}) == dangerZone {
			isMoveSafe["left"] = false
		} else if (Coord{X: myHead.X, Y: myHead.Y + 1}) == dangerZone {
			isMoveSafe["up"] = false
		} else if (Coord{X: myHead.X, Y: myHead.Y - 1}) == dangerZone {
			isMoveSafe["down"] = false
		}
	}

	// Are there any safe moves left?
	safeMoves := []string{}
	for move, isSafe := range isMoveSafe {
		if isSafe {
			safeMoves = append(safeMoves, move)
		}
	}

	if len(safeMoves) == 0 {
		// If no safe moves, consider potential head-to-head moves
		headToHeadMoves := []string{}
		for move, isSafe := range isMoveSafe {
			if !isSafe {
				if potentialHeadToHead, exists := map[string]Coord{
					"up":    {X: myHead.X, Y: myHead.Y + 1},
					"down":  {X: myHead.X, Y: myHead.Y - 1},
					"left":  {X: myHead.X - 1, Y: myHead.Y},
					"right": {X: myHead.X + 1, Y: myHead.Y},
				}[move]; exists {
					for _, potential := range potentialHeadToHeads {
						if potential == potentialHeadToHead {
							headToHeadMoves = append(headToHeadMoves, move)
						}
					}
				}
			}
		}

		// Prioritize head-to-head moves based on the opponent's proximity to food
		if len(headToHeadMoves) > 0 {
			food := state.Board.Food
			var bestMove string
			minFoodDistance := math.MaxInt32

			for _, move := range headToHeadMoves {
				potentialMove := map[string]Coord{
					"up":    {X: myHead.X, Y: myHead.Y + 1},
					"down":  {X: myHead.X, Y: myHead.Y - 1},
					"left":  {X: myHead.X - 1, Y: myHead.Y},
					"right": {X: myHead.X + 1, Y: myHead.Y},
				}[move]

				for _, foodPoint := range food {
					distance := ManhattanDistance(potentialMove, foodPoint)
					if distance < minFoodDistance {
						minFoodDistance = distance
						bestMove = move
					}
				}
			}

			log.Printf("MOVE %d: Head-to-head move %s\n", state.Turn, bestMove)
			return BattlesnakeMoveResponse{Move: bestMove}
		}

		log.Printf("MOVE %d: No safe moves detected :( Moving up\n", state.Turn)
		return BattlesnakeMoveResponse{Move: "up"}
	}

	// Choose a random move from the safe ones
	nextMove := safeMoves[rand.Intn(len(safeMoves))]

	// Move towards the closest food if it's safe
	food := state.Board.Food
	closestFood := FindClosestFood(myHead, food)

	dx := closestFood.X - myHead.X
	dy := closestFood.Y - myHead.Y

	var goodmove string

	if dx > 0 {
		goodmove = "right"
	} else if dx < 0 {
		goodmove = "left"
	} else if dy > 0 {
		goodmove = "up"
	} else if dy < 0 {
		goodmove = "down"
	}

	if isMoveSafe[goodmove] {
		log.Printf("MOVE %d: %s\n", state.Turn, goodmove)
		return BattlesnakeMoveResponse{Move: goodmove}
	}

	log.Printf("MOVE %d: %s\n", state.Turn, nextMove)
	return BattlesnakeMoveResponse{Move: nextMove}
}

func main() {
	RunServer()
}
