package Project;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;


public class main extends Application {

    @Override
    public void start(Stage stage) throws IOException {
        FXMLLoader fxmlLoader = new FXMLLoader(PleaseProvideControllerClassName.class.getResource("D:\\GitRepos\\tsp-proj\\src\\main\\resources\\Project\\template.fxml"));
        Scene scene = new Scene(fxmlLoader.load());
        stage.setTitle("Smart Stool");
        stage.setScene(scene);
        stage.show();

    }
    public static void main(String[] args) {
        launch();
    }
}
