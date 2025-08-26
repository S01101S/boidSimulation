#include <iostream>
#include <random>
#include <SFML/Graphics.hpp>
#include <vector>



class Boid
{
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    sf::ConvexShape shape;

    Boid(float x, float y, std::mt19937 generator)
    {
        std::uniform_real_distribution<float> distribution(-2.f, 2.f);


        position = sf::Vector2f(x, y);
        velocity = sf::Vector2f(distribution(generator), distribution(generator));
        acceleration = sf::Vector2f(0.f, 0.f);

        const float sizeScale = generateSize(generator);
        sf::Vector2f newSize (15.f * sizeScale, 2.f * sizeScale);
        shape.setPointCount(3);
        shape.setPoint(0,sf::Vector2f(10, 0));
        shape.setPoint(1, sf::Vector2f(-5, -5));
        shape.setPoint(2, sf::Vector2f(-5, 5));
        shape.setOrigin(0, 0);
        shape.setScale(sizeScale, sizeScale);
        shape.setFillColor(sf::Color(generateColor(generator), generateColor(generator), generateColor(generator)));
        shape.setPosition(position);
    }

    int generateColor(std::mt19937& generator)
    {
        std::uniform_int_distribution<int> distribution(0, 255);
        return distribution(generator);
    }

    float generateSize(std::mt19937& generator)
    {
        std::uniform_real_distribution<float> distribution(0.8f, 1.2f);
        return distribution(generator);
    }

    void updateBoidPosition(float windowWidth, float windowHeight)
    {
        velocity += acceleration;

        const float velocityMagnitude = sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y));

        if (velocityMagnitude > 5.f)
        {
            velocity = (velocity / velocityMagnitude) * 5.f;
        }

        position += velocity;
        acceleration *= 0.f;

        float rotationAngle = atan2(velocity.y, velocity.x) * (180.f / M_PI);

        shape.setPosition(position);
        shape.setRotation(rotationAngle);

        if (position.x < 0)
        {
            position.x += windowWidth;
        }
        else if (position.x > windowWidth)
        {
            position.x -= windowWidth;
        }

        if (position.y < 0)
        {
            position.y += windowHeight;
        }
        else if (position.y > windowHeight)
        {
            position.y -= windowHeight;
        }
    }

    void flockRules(std::vector<Boid>& boids, sf::Vector2f mousePosition)
    {
        sf::Vector2f totalPosition = sf::Vector2f(0.f, 0.f);
        sf::Vector2f totalVelocity = sf::Vector2f(0.f, 0.f);

        const float mouseDx = mousePosition.x - this->position.x;
        const float mouseDy = mousePosition.y - this->position.y;
        float mouseDistance = sqrt((mouseDx * mouseDx) + (mouseDy * mouseDy));

        if (mouseDistance < 100.f)
        {
            sf::Vector2f fleeDirection = this->position - mousePosition;
            float magnitudeOfFleeDirection = sqrt((fleeDirection.x * fleeDirection.x) + (fleeDirection.y * fleeDirection.y));

            if (magnitudeOfFleeDirection > 0)
            {
                sf::Vector2f normalisedFleeingdirection = fleeDirection / magnitudeOfFleeDirection;
                normalisedFleeingdirection *= 1.f;
                this->acceleration += normalisedFleeingdirection;
            }
        }
        else
        {
            for (auto& boid : boids)
            {
                totalPosition += boid.position;
            }

            sf::Vector2f centerOfFlock = totalPosition / static_cast<float>(boids.size());
            sf::Vector2f steeringDirection = centerOfFlock - position;
            float magnitudeOfSteeringDirection = sqrt(steeringDirection.x*steeringDirection.x + steeringDirection.y*steeringDirection.y);
            sf::Vector2f normalisedSteeringDirectionForce = steeringDirection / magnitudeOfSteeringDirection;

            normalisedSteeringDirectionForce *= 0.05f;
            acceleration += normalisedSteeringDirectionForce;

            for (auto& neighbourBoid : boids)
            {
                if (this == &neighbourBoid)
                {
                    continue;
                }

                float dx = (this->position.x - neighbourBoid.position.x);
                float dy = (this->position.y - neighbourBoid.position.y);
                float distance = sqrt((dx * dx) + (dy * dy));

                if (distance < 25.f)
                {
                    sf::Vector2f steerAwayDirection = this->position - neighbourBoid.position;
                    float magnitudeOfSteerAwayDirection = sqrt(steerAwayDirection.x * steerAwayDirection.x + steerAwayDirection.y * steerAwayDirection.y);
                    sf::Vector2f normalisedSteerAwayDirection = steerAwayDirection / magnitudeOfSteerAwayDirection;
                    normalisedSteerAwayDirection *= 0.05f;
                    this->acceleration += normalisedSteerAwayDirection;
                }
            }

            for (auto& boid : boids)
            {
                totalVelocity += boid.velocity;
            }

            sf::Vector2f averageVelocity = totalVelocity / static_cast<float>(boids.size());
            sf::Vector2f velocitySteering = averageVelocity - this->velocity;
            float magnitudeOfVelocitySteering = sqrt(velocitySteering.x * velocitySteering.x + velocitySteering.y * velocitySteering.y);
            if (magnitudeOfSteeringDirection > 0)
            {
                sf::Vector2f normalisedVelocitySteering = velocitySteering / magnitudeOfVelocitySteering;
                normalisedVelocitySteering *= 0.05f;
                this->acceleration += normalisedVelocitySteering;
            }
        }

        const float accelerationMagnitude = sqrt((this->acceleration.x * this->acceleration.x) + (this->acceleration.y * this->acceleration.y));

        if (accelerationMagnitude > 0.5f)
        {
            this->acceleration = (this->acceleration / accelerationMagnitude) * 0.5f;
        }

    }
};

int main()
{
    const float windowHeight = 600.f;
    const float windowWidth = 800.f;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> distributionX(0, windowWidth);
    std::uniform_real_distribution<float> distributionY(0, windowHeight);


    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boid Simulation");
    window.setFramerateLimit(60);

    std::vector<Boid> boidList;

    for (int i=0; i<100; i++)
    {
        const float boidXPosition = distributionX(generator);
        const float boidYPosition = distributionY(generator);
        Boid newBoid(boidXPosition, boidYPosition, generator);
        boidList.push_back(newBoid);
    }



    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        sf::Vector2i pixelPosition = sf::Mouse::getPosition(window);
        sf::Vector2f mousePosition = window.mapPixelToCoords(pixelPosition);

        window.clear(sf::Color::Black);

        for (auto& currentBoid : boidList)
        {
            currentBoid.flockRules(boidList, mousePosition);
            currentBoid.updateBoidPosition(windowWidth, windowHeight);
            window.draw(currentBoid.shape);
        }

        window.display();


    }

    return 0;

}